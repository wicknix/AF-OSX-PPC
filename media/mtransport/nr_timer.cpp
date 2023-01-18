/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

// Original code by: ekr@rtfm.com

// Implementation of the NR timer interface

// Some code here copied from nrappkit. The license was.

/**
   Copyright (C) 2004, Network Resonance, Inc.
   Copyright (C) 2006, Network Resonance, Inc.
   All Rights Reserved

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
   3. Neither the name of Network Resonance, Inc. nor the name of any
      contributors to this software may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.


   ekr@rtfm.com  Sun Feb 22 19:35:24 2004
 */

#include <string>

#include "nsCOMPtr.h"
#include "nsComponentManagerUtils.h"
#include "nsServiceManagerUtils.h"
#include "nsIEventTarget.h"
#include "nsITimer.h"
#include "nsNetCID.h"
#include "runnable_utils.h"

extern "C" {
#include "nr_api.h"
#include "async_timer.h"
}


namespace mozilla {

class nrappkitTimerCallback : public nsITimerCallback
{
public:
  // We're going to release ourself in the callback, so we need to be threadsafe
  NS_DECL_THREADSAFE_ISUPPORTS
  NS_DECL_NSITIMERCALLBACK

  nrappkitTimerCallback(NR_async_cb cb, void *cb_arg,
                        const char *function, int line)
    : cb_(cb), cb_arg_(cb_arg), function_(function), line_(line) {
  }

private:
  virtual ~nrappkitTimerCallback() {}

protected:
  /* additional members */
  NR_async_cb cb_;
  void *cb_arg_;
  std::string function_;
  int line_;
};

NS_IMPL_ISUPPORTS(nrappkitTimerCallback, nsITimerCallback)

NS_IMETHODIMP nrappkitTimerCallback::Notify(nsITimer *timer) {
  r_log(LOG_GENERIC, LOG_DEBUG, "Timer callback fired (set in %s:%d)",
        function_.c_str(), line_);

  cb_(0, 0, cb_arg_);

  // Allow the timer to go away.
  timer->Release();
  return NS_OK;
}
}  // close namespace


using namespace mozilla;

// These timers must only be used from the STS thread.
// This function is a helper that enforces that.
static void CheckSTSThread() {
  nsresult rv;

  nsCOMPtr<nsIEventTarget> sts_thread;

  sts_thread = do_GetService(NS_SOCKETTRANSPORTSERVICE_CONTRACTID, &rv);

  MOZ_ASSERT(NS_SUCCEEDED(rv));
  ASSERT_ON_THREAD(sts_thread);
}

int NR_async_timer_set(int timeout, NR_async_cb cb, void *arg, char *func,
                       int l, void **handle) {
  nsresult rv;
  CheckSTSThread();

  nsCOMPtr<nsITimer> timer = do_CreateInstance(NS_TIMER_CONTRACTID, &rv);
  if (NS_FAILED(rv)) {
    return(R_FAILED);
  }

  rv = timer->InitWithCallback(new nrappkitTimerCallback(cb, arg, func, l),
                               timeout, nsITimer::TYPE_ONE_SHOT);
  if (NS_FAILED(rv)) {
    return R_FAILED;
  }

  // We need an AddRef here to keep the timer alive, per the spec.
  timer->AddRef();

  if (handle)
    *handle = timer.get();
  // Bug 818806: if we have no handle to the timer, we have no way to avoid
  // it leaking (though not the callback object) if it never fires (or if
  // we exit before it fires).

  return 0;
}

int NR_async_schedule(NR_async_cb cb, void *arg, char *func, int l) {
  // No need to check the thread because we check it next in the
  // timer set.
  return NR_async_timer_set(0, cb, arg, func, l, nullptr);
}

int NR_async_timer_cancel(void *handle) {
  // Check for the handle being nonzero because sometimes we get
  // no-op cancels that aren't on the STS thread. This can be
  // non-racy as long as the upper-level code is careful.
  if (!handle)
    return 0;

  CheckSTSThread();

  nsITimer *timer = static_cast<nsITimer *>(handle);

  timer->Cancel();
  // Allow the timer to go away.
  timer->Release();

  return 0;
}

