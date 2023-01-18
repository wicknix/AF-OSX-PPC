/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(GMPAudioDecoder_h_)
#define GMPAudioDecoder_h_

#include "GMPAudioDecoderProxy.h"
#include "MediaDataDecoderProxy.h"
#include "PlatformDecoderModule.h"
#include "mozIGeckoMediaPluginService.h"

namespace mozilla {

class AudioCallbackAdapter : public GMPAudioDecoderCallbackProxy {
public:
  explicit AudioCallbackAdapter(MediaDataDecoderCallbackProxy* aCallback)
   : mCallback(aCallback)
   , mLastStreamOffset(0)
   , mAudioFrameSum(0)
   , mAudioFrameOffset(0)
   , mMustRecaptureAudioPosition(true)
  {}

  // GMPAudioDecoderCallbackProxy
  virtual void Decoded(const nsTArray<int16_t>& aPCM, uint64_t aTimeStamp, uint32_t aChannels, uint32_t aRate) override;
  virtual void InputDataExhausted() override;
  virtual void DrainComplete() override;
  virtual void ResetComplete() override;
  virtual void Error(GMPErr aErr) override;
  virtual void Terminated() override;

  void SetLastStreamOffset(int64_t aStreamOffset) {
    mLastStreamOffset = aStreamOffset;
  }

private:
  MediaDataDecoderCallbackProxy* mCallback;
  int64_t mLastStreamOffset;

  int64_t mAudioFrameSum;
  int64_t mAudioFrameOffset;
  bool mMustRecaptureAudioPosition;
};

class GMPAudioDecoder : public MediaDataDecoder {
protected:
  GMPAudioDecoder(const mp4_demuxer::AudioDecoderConfig& aConfig,
                  MediaTaskQueue* aTaskQueue,
                  MediaDataDecoderCallbackProxy* aCallback,
                  AudioCallbackAdapter* aAdapter)
   : mConfig(aConfig)
   , mCallback(aCallback)
   , mGMP(nullptr)
   , mAdapter(aAdapter)
  {
  }

public:
  GMPAudioDecoder(const mp4_demuxer::AudioDecoderConfig& aConfig,
                  MediaTaskQueue* aTaskQueue,
                  MediaDataDecoderCallbackProxy* aCallback)
   : mConfig(aConfig)
   , mCallback(aCallback)
   , mGMP(nullptr)
   , mAdapter(new AudioCallbackAdapter(aCallback))
  {
  }

  virtual nsresult Init() override;
  virtual nsresult Input(mp4_demuxer::MP4Sample* aSample) override;
  virtual nsresult Flush() override;
  virtual nsresult Drain() override;
  virtual nsresult Shutdown() override;

protected:
  virtual void InitTags(nsTArray<nsCString>& aTags);
  virtual nsCString GetNodeId();

private:
  const mp4_demuxer::AudioDecoderConfig& mConfig;
  MediaDataDecoderCallbackProxy* mCallback;
  nsCOMPtr<mozIGeckoMediaPluginService> mMPS;
  GMPAudioDecoderProxy* mGMP;
  nsAutoPtr<AudioCallbackAdapter> mAdapter;
};

} // namespace mozilla

#endif // GMPAudioDecoder_h_
