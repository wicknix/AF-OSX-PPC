/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_AppleDecoderModule_h
#define mozilla_AppleDecoderModule_h

#include "PlatformDecoderModule.h"

namespace mozilla {

class AppleDecoderModule : public PlatformDecoderModule {
public:
  AppleDecoderModule();
  virtual ~AppleDecoderModule();

  virtual nsresult Startup() override;

  // Decode thread.
  virtual already_AddRefed<MediaDataDecoder>
  CreateVideoDecoder(const mp4_demuxer::VideoDecoderConfig& aConfig,
                     layers::LayersBackend aLayersBackend,
                     layers::ImageContainer* aImageContainer,
                     FlushableMediaTaskQueue* aVideoTaskQueue,
                     MediaDataDecoderCallback* aCallback) override;

  // Decode thread.
  virtual already_AddRefed<MediaDataDecoder>
  CreateAudioDecoder(const mp4_demuxer::AudioDecoderConfig& aConfig,
                     FlushableMediaTaskQueue* aAudioTaskQueue,
                     MediaDataDecoderCallback* aCallback) override;

  virtual bool SupportsAudioMimeType(const nsACString& aMimeType) override;
  virtual bool
  DecoderNeedsAVCC(const mp4_demuxer::VideoDecoderConfig& aConfig) override;

  static void Init();
  static nsresult CanDecode();

private:
  friend class InitTask;
  friend class LinkTask;
  friend class UnlinkTask;

  static bool sInitialized;
  static bool sIsVTAvailable;
  static bool sIsVTHWAvailable;
  static bool sIsVDAAvailable;
  static bool sForceVDA;
};

} // namespace mozilla

#endif // mozilla_AppleDecoderModule_h
