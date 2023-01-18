/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MOZ_UNWINDER_THREAD_2_H
#define MOZ_UNWINDER_THREAD_2_H

#include "GeckoProfilerImpl.h"
#include "ProfileEntry.h"

#include "PlatformMacros.h"
#if defined(SPS_OS_android) || defined(SPS_OS_linux)
# include "LulMain.h"
#endif

/* Top level exports of UnwinderThread.cpp. */

// Abstract type.  A buffer which is used to transfer information between
// the sampled thread(s) and the unwinder thread(s).
typedef
  struct _UnwinderThreadBuffer 
  UnwinderThreadBuffer;

// RUNS IN SIGHANDLER CONTEXT
// Called in the sampled thread (signal) context.  Adds a ProfileEntry
// into an UnwinderThreadBuffer that the thread has previously obtained
// by a call to utb__acquire_empty_buffer.
void utb__addEntry(/*MOD*/UnwinderThreadBuffer* utb,
                   ProfileEntry ent);

// Create the unwinder thread.  At the moment there can be only one.
void uwt__init();

// Request the unwinder thread to exit, and wait until it has done so.
// This must be called before stopping the profiler because we hold a
// reference to the profile which is owned by the profiler.
void uwt__stop();

// Release the unwinder resources. This must be called after profiling
// has stop. At this point we know the profiler doesn't hold any buffer
// and can safely release any resources.
void uwt__deinit();

// Registers a sampler thread for profiling.  Threads must be
// registered before calls to call utb__acquire_empty_buffer or
// utb__release_full_buffer have any effect.  If stackTop is
// nullptr, the call is ignored.
void uwt__register_thread_for_profiling(void* stackTop);

// Deregister a sampler thread for profiling.
void uwt__unregister_thread_for_profiling();

// RUNS IN SIGHANDLER CONTEXT 
// Called in the sampled thread (signal) context.  Get an empty buffer
// into which ProfileEntries can be put.  It may return nullptr if no
// empty buffers can be found, which will be the case if the unwinder
// thread(s) have fallen behind for some reason.  In this case the
// sampled thread must simply give up and return from the signal
// handler immediately, else it risks deadlock.
//
// If the calling thread has not previously registered itself for
// profiling via uwt__register_thread_for_profiling, this routine
// returns nullptr.
UnwinderThreadBuffer* uwt__acquire_empty_buffer();

// RUNS IN SIGHANDLER CONTEXT
// Called in the sampled thread (signal) context.  Release a buffer
// that the sampled thread has acquired, handing the contents to
// the unwinder thread, and, if necessary, passing sufficient
// information (stack top chunk, + registers) to also do a native
// unwind.  If 'ucV' is nullptr, no native unwind is done.  If non-nullptr,
// it is assumed to point to a ucontext_t* that holds the initial 
// register state for the unwind.  The results of all of this are
// dumped into |aProfile| (by the unwinder thread, not the calling thread).
void uwt__release_full_buffer(ThreadProfile* aProfile,
                              UnwinderThreadBuffer* utb,
                              void* /* ucontext_t*, really */ ucV);

struct LinkedUWTBuffer;

// Get an empty buffer for synchronous unwinding.
// This function is NOT signal-safe.
LinkedUWTBuffer* utb__acquire_sync_buffer(void* stackTop);

void utb__finish_sync_buffer(ThreadProfile* aProfile,
                             UnwinderThreadBuffer* utb,
                             void* /* ucontext_t*, really */ ucV);

// Free an empty buffer that was previously allocated by
// utb__acquire_sync_buffer.
void utb__release_sync_buffer(LinkedUWTBuffer* utb);

// Unwind complete, mark a synchronous unwind buffer as empty
void utb__end_sync_buffer_unwind(LinkedUWTBuffer* utb);

// This typedef must match uwt__release_full_buffer and uwt__finish_sync_buffer
typedef void (*UTB_RELEASE_FUNC)(ThreadProfile*,UnwinderThreadBuffer*,void*);

#if defined(SPS_OS_android) || defined(SPS_OS_linux)
// Notify |aLUL| of the objects in the current process, so as to get
// it to read unwind data for them.  This has to be externally visible
// so it can be used in LUL unit tests, tools/profiler/tests/gtest/LulTest.cpp.
void read_procmaps(lul::LUL* aLUL);
#endif

#endif /* ndef MOZ_UNWINDER_THREAD_2_H */
