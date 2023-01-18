/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __PROFILER_BACKTRACE_H
#define __PROFILER_BACKTRACE_H

class SyncProfile;

class ProfilerBacktrace
{
public:
  explicit ProfilerBacktrace(SyncProfile* aProfile);
  ~ProfilerBacktrace();

  void StreamJSObject(JSStreamWriter& b);

private:
  ProfilerBacktrace(const ProfilerBacktrace&);
  ProfilerBacktrace& operator=(const ProfilerBacktrace&);

  SyncProfile*  mProfile;
};

#endif // __PROFILER_BACKTRACE_H

