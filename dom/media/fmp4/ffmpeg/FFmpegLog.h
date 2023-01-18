/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __FFmpegLog_h__
#define __FFmpegLog_h__

#ifdef PR_LOGGING
extern PRLogModuleInfo* GetFFmpegDecoderLog();
#define FFMPEG_LOG(...) PR_LOG(GetFFmpegDecoderLog(), PR_LOG_DEBUG, (__VA_ARGS__))
#else
#define FFMPEG_LOG(...)
#endif

#endif // __FFmpegLog_h__
