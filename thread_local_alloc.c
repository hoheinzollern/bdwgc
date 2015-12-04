/*
 * Copyright (c) 2000-2005 by Hewlett-Packard Company.  All rights reserved.
 *
 * THIS MATERIAL IS PROVIDED AS IS, WITH ABSOLUTELY NO WARRANTY EXPRESSED
 * OR IMPLIED.  ANY USE IS AT YOUR OWN RISK.
 *
 * Permission is hereby granted to use or copy this program
 * for any purpose,  provided the above notices are retained on all copies.
 * Permission to modify the code and to distribute modified code is granted,
 * provided the above notices are retained, and a notice that the code was
 * modified is included with the above copyright notice.
 */

#include "private/gc_priv.h"
#include "gc_typed.h"
#include "gc_inline.h"

#ifdef GC_GCJ_SUPPORT
# include "gc_gcj.h"
#endif

#if defined(THREAD_LOCAL_ALLOC)

#ifndef THREADS
# error "invalid config - THREAD_LOCAL_ALLOC requires GC_THREADS"
#endif

#include "private/thread_local_alloc.h"

#include <stdlib.h>

#if defined(USE_COMPILER_TLS)
  __thread GC_ATTR_TLS_FAST
#elif defined(USE_WIN32_COMPILER_TLS)
  __declspec(thread) GC_ATTR_TLS_FAST
#endif
GC_key_t GC_thread_key;

static GC_bool keys_initialized;

#ifdef ENABLE_DISCLAIM
  GC_INNER ptr_t * GC_finalized_objfreelist = NULL;
        /* This variable is declared here to prevent linking of         */
        /* fnlz_mlc module unless the client uses the latter one.       */
#endif

/* Return a single nonempty freelist fl to the global one pointed to    */
/* by gfl.                                                              */

static void return_single_freelist(void *fl, void **gfl)
{
    void *q, **qptr;

    if (*gfl == 0) {
      *gfl = fl;
    } else {
      GC_ASSERT(GC_size(fl) == GC_size(*gfl));
      /* Concatenate: */
        qptr = &(obj_link(fl));
        while ((word)(q = *qptr) >= HBLKSIZE)
          qptr = &(obj_link(q));
        GC_ASSERT(0 == q);
        *qptr = *gfl;
        *gfl = fl;
    }
}

/* Recover the contents of the freelist array fl into the global one gfl.*/
/* We hold the allocator lock.                                          */
static void return_freelists(void **fl, void **gfl)
{
    int i;

    for (i = 1; i < TINY_FREELISTS; ++i) {
        if ((word)(fl[i]) >= HBLKSIZE) {
          return_single_freelist(fl[i], gfl+i);
        }
        /* Clear fl[i], since the thread structure may hang around.     */
        /* Do it in a way that is likely to trap if we access it.       */
        fl[i] = (ptr_t)HBLKSIZE;
    }
    /* The 0 granule freelist really contains 1 granule objects.        */
#   ifdef GC_GCJ_SUPPORT
    if (fl[0] == ERROR_FL) return;
#   endif
    if ((word)(fl[0]) >= HBLKSIZE) {
        return_single_freelist(fl[0], gfl+1);
    }
}

#ifdef USE_PTHREAD_SPECIFIC
  /* Re-set the TLS value on thread cleanup to allow thread-local       */
  /* allocations to happen in the TLS destructors.                      */
  /* GC_unregister_my_thread (and similar routines) will finally set    */
  /* the GC_thread_key to NULL preventing this destructor from being    */
  /* called repeatedly.                                                 */
  static void reset_thread_key(void* v) {
    pthread_setspecific(GC_thread_key, v);
  }
#else
# define reset_thread_key 0
#endif

/* Each thread structure must be initialized.   */
/* This call must be made from the new thread.  */
GC_INNER void GC_init_thread_local(GC_tlfs p)
{
  int i, j;

    GC_ASSERT(I_HOLD_LOCK());
    if (!EXPECT(keys_initialized, TRUE)) {
        GC_ASSERT((word)&GC_thread_key % sizeof(word) == 0);
        if (0 != GC_key_create(&GC_thread_key, reset_thread_key)) {
            ABORT("Failed to create key for local allocator");
        }
        keys_initialized = TRUE;
    }
    if (0 != GC_setspecific(GC_thread_key, p)) {
        ABORT("Failed to set thread specific allocation pointers");
    }
    for (i = 0; i < MAXOBJKINDS; ++i) {
        for (j = 1; j < TINY_FREELISTS; ++j) {
            p->freelists[i][j] = (void *)(word)1;
        }
        /* Set up the size 0 free lists.    */
        /* We now handle most of them like regular free lists, to ensure    */
        /* That explicit deallocation works.  However, allocation of a      */
        /* size 0 "gcj" object is always an error.                          */
#   ifdef GC_GCJ_SUPPORT
        if (i == GC_gcj_kind) {
          p->freelists[i][0] = ERROR_FL;
        } else {
#   endif
          p->freelists[i][0] = (void *)(word)1;
#   ifdef GC_GCJ_SUPPORT
        }
#   endif
    }
#   ifdef ENABLE_DISCLAIM
    for (i = 0; i < TINY_FREELISTS; ++i) {
        p -> finalized_freelists[i] = (void *)(word)1;
    }
    p->finalized_freelists[0] = (void *)(word)1;
#   endif
}

/* We hold the allocator lock.  */
GC_INNER void GC_destroy_thread_local(GC_tlfs p)
{
  int i;
  /* We currently only do this from the thread itself or from */
  /* the fork handler for a child process.                    */
  for (i = 0; i < MAXOBJKINDS; ++i) {
    return_freelists(p->freelists[i], GC_freelist[i]);
  }
#   ifdef ENABLE_DISCLAIM
    return_freelists(p -> finalized_freelists,
        (void **)GC_finalized_objfreelist);
#   endif
}

#ifdef GC_GCJ_SUPPORT

# include "atomic_ops.h" /* for AO_compiler_barrier() */

/* Gcj-style allocation without locks is extremely tricky.  The         */
/* fundamental issue is that we may end up marking a free list, which   */
/* has freelist links instead of "vtable" pointers.  That is usually    */
/* OK, since the next object on the free list will be cleared, and      */
/* will thus be interpreted as containing a zero descriptor.  That's    */
/* fine if the object has not yet been initialized.  But there are      */
/* interesting potential races.                                         */
/* In the case of incremental collection, this seems hopeless, since    */
/* the marker may run asynchronously, and may pick up the pointer to    */
/* the next freelist entry (which it thinks is a vtable pointer), get   */
/* suspended for a while, and then see an allocated object instead      */
/* of the vtable.  This may be avoidable with either a handshake with   */
/* the collector or, probably more easily, by moving the free list      */
/* links to the second word of each object.  The latter isn't a         */
/* universal win, since on architecture like Itanium, nonzero offsets   */
/* are not necessarily free.  And there may be cache fill order issues. */
/* For now, we punt with incremental GC.  This probably means that      */
/* incremental GC should be enabled before we fork a second thread.     */
/* Unlike the other thread local allocation calls, we assume that the   */
/* collector has been explicitly initialized.                           */
GC_API GC_ATTR_MALLOC void * GC_CALL GC_gcj_malloc(size_t bytes,
                                                   void * ptr_to_struct_containing_descr)
{
  if (EXPECT(GC_incremental, FALSE)) {
    return GC_core_gcj_malloc(bytes, ptr_to_struct_containing_descr);
  } else {
    size_t granules = ROUNDED_UP_GRANULES(bytes);
    void *result;
    void **tiny_fl = ((GC_tlfs)GC_getspecific(GC_thread_key))
      -> freelists[GC_gcj_kind];
    GC_ASSERT(GC_gcj_malloc_initialized);
    GC_FAST_MALLOC_GRANS(result, granules, tiny_fl, DIRECT_GRANULES,
                         GC_gcj_kind,
                         GC_core_gcj_malloc(bytes,
                                            ptr_to_struct_containing_descr),
                         {AO_compiler_barrier();
                           *(void **)result = ptr_to_struct_containing_descr;});
        /* This forces the initialization of the "method ptr".          */
        /* This is necessary to ensure some very subtle properties      */
        /* required if a GC is run in the middle of such an allocation. */
        /* Here we implicitly also assume atomicity for the free list.  */
        /* and method pointer assignments.                              */
        /* We must update the freelist before we store the pointer.     */
        /* Otherwise a GC at this point would see a corrupted           */
        /* free list.                                                   */
        /* A real memory barrier is not needed, since the               */
        /* action of stopping this thread will cause prior writes       */
        /* to complete.                                                 */
        /* We assert that any concurrent marker will stop us.           */
        /* Thus it is impossible for a mark procedure to see the        */
        /* allocation of the next object, but to see this object        */
        /* still containing a free list pointer.  Otherwise the         */
        /* marker, by misinterpreting the freelist link as a vtable     */
        /* pointer, might find a random "mark descriptor" in the next   */
        /* object.                                                      */
    return result;
  }
}

#endif /* GC_GCJ_SUPPORT */

/* The thread support layer must arrange to mark thread-local   */
/* free lists explicitly, since the link field is often         */
/* invisible to the marker.  It knows how to find all threads;  */
/* we take care of an individual thread freelist structure.     */
GC_INNER void GC_mark_thread_local_fls_for(GC_tlfs p)
{
    ptr_t q;
    int i, j;

    for (i = 0; i < MAXOBJKINDS; ++i) {
        for (j = 0; j < TINY_FREELISTS; ++j) {
            q = p->freelists[i][j];
#     ifdef GC_GCJ_SUPPORT
            if (j > 0) {
                if ((word)q > HBLKSIZE) GC_set_fl_marks(q);
            } else {
#     endif /* GC_GCJ_SUPPORT */
                if ((word)q > HBLKSIZE) GC_set_fl_marks(q);
#     ifdef GC_GCJ_SUPPORT
            }
#     endif /* GC_GCJ_SUPPORT */
        }
    }
}

#if defined(GC_ASSERTIONS)
/* Check that all thread-local free-lists in p are completely marked. */
void GC_check_tls_for(GC_tlfs p)
{
    int i, j;

    for (j = 1; j < TINY_FREELISTS; ++j) {
        for (i = 0; i < MAXOBJKINDS; ++i) {
            GC_check_fl_marks(&p->freelists[i][j]);
        }
#     ifdef ENABLE_DISCLAIM
        GC_check_fl_marks(&p->finalized_freelists[j]);
#     endif
    }
}
#endif /* GC_ASSERTIONS */

#endif /* THREAD_LOCAL_ALLOC */
