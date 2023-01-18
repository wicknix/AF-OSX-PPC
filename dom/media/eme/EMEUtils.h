/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef EME_LOG_H_
#define EME_LOG_H_

#include "prlog.h"
#include "nsString.h"

namespace mozilla {

#ifdef PR_LOGGING

  #ifndef EME_LOG
    PRLogModuleInfo* GetEMELog();
    #define EME_LOG(...) PR_LOG(GetEMELog(), PR_LOG_DEBUG, (__VA_ARGS__))
  #endif

  #ifndef EME_VERBOSE_LOG
    PRLogModuleInfo* GetEMEVerboseLog();
    #define EME_VERBOSE_LOG(...) PR_LOG(GetEMEVerboseLog(), PR_LOG_DEBUG, (__VA_ARGS__))
  #else
    #ifndef EME_LOG
      #define EME_LOG(...)
    #endif

    #ifndef EME_VERBOSE_LOG
      #define EME_VERBOSE_LOG(...)
    #endif
  #endif

#else

  #ifndef EME_LOG
    #define EME_LOG(...)
  #endif

  #ifndef EME_VERBOSE_LOG
    #define EME_VERBOSE_LOG(...)
  #endif

#endif // PR_LOGGING

#define NO_CDM_VERSION -1

// Checks a keySystem string against a whitelist, and determines whether
// the keySystem is in the whitelist, and extracts the requested minimum
// CDM version.
//
// Format of EME keysystems:
// com.domain.keysystem[.minVersionAsInt]
// i.e. org.w3.clearkey, com.adobe.primetime.7
//
// Returns true if aKeySystem contains a valid keySystem which we support,
// false otherwise.
//
// On success, aOutKeySystem contains the keySystem string stripped of the
// min version number, and aOutMinCDMVersion contains the min version number
// if present. If it was not present, aOutMinCDMVersion is NO_CDM_VERSION.
bool ParseKeySystem(const nsAString& aKeySystem,
                    nsAString& aOutKeySystem,
                    int32_t& aOutMinCDMVersion);

} // namespace mozilla

#endif // EME_LOG_H_
