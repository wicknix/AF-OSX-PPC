/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "CacheLog.h"

namespace mozilla {
namespace net {

#if defined(PR_LOGGING)
// Log module for cache2 (2013) cache implementation logging...
//
// To enable logging (see prlog.h for full details):
//
//    set NSPR_LOG_MODULES=cache2:5
//    set NSPR_LOG_FILE=nspr.log
//
// this enables PR_LOG_DEBUG level information and places all output in
// the file nspr.log
PRLogModuleInfo* GetCache2Log()
{
  static PRLogModuleInfo *sLog;
  if (!sLog)
    sLog = PR_NewLogModule("cache2");
  return sLog;
}
#endif

} // net
} // mozilla
