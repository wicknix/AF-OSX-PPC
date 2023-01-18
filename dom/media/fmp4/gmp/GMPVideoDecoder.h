/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(GMPVideoDecoder_h_)
#define GMPVideoDecoder_h_

#include "GMPVideoDecoderProxy.h"
#include "ImageContainer.h"
#include "MediaDataDecoderProxy.h"
#include "PlatformDecoderModule.h"
#include "mozIGeckoMediaPluginService.h"
#include "mp4_demuxer/DecoderData.h"

namespace mozilla {

class VideoCallbackAdapter : public GMPVideoDecoderCallbackProxy {
public:
  VideoCallbackAdapter(MediaDataDecoderCallbackProxy* aCallback,
                       VideoInfo aVideoInfo,
                       layers::ImageContainer* aImageContainer)
   : mCallback(aCallback)
   , mLastStreamOffset(0)
   , mVideoInfo(aVideoInfo)
   , mImageContainer(aImageContainer)
  {}

  // GMPVideoDecoderCallbackProxy
  virtual void Decoded(GMPVideoi420Frame* aDecodedFrame) override;
  virtual void ReceivedDecodedReferenceFrame(const uint64_t aPictureId) override;
  virtual void ReceivedDecodedFrame(const uint64_t aPictureId) override;
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

  VideoInfo mVideoInfo;
  nsRefPtr<layers::ImageContainer> mImageContainer;
};

class GMPVideoDecoder : public MediaDataDecoder {
protected:
  GMPVideoDecoder(const mp4_demuxer::VideoDecoderConfig& aConfig,
                  layers::LayersBackend aLayersBackend,
                  layers::ImageContainer* aImageContainer,
                  MediaTaskQueue* aTaskQueue,
                  MediaDataDecoderCallbackProxy* aCallback,
                  VideoCallbackAdapter* aAdapter)
   : mConfig(aConfig)
   , mCallback(aCallback)
   , mGMP(nullptr)
   , mHost(nullptr)
   , mAdapter(aAdapter)
   , mConvertNALUnitLengths(false)
  {
  }

public:
  GMPVideoDecoder(const mp4_demuxer::VideoDecoderConfig& aConfig,
                  layers::LayersBackend aLayersBackend,
                  layers::ImageContainer* aImageContainer,
                  MediaTaskQueue* aTaskQueue,
                  MediaDataDecoderCallbackProxy* aCallback)
   : mConfig(aConfig)
   , mCallback(aCallback)
   , mGMP(nullptr)
   , mHost(nullptr)
   , mAdapter(new VideoCallbackAdapter(aCallback,
                                       VideoInfo(aConfig.display_width,
                                                 aConfig.display_height),
                                       aImageContainer))
   , mConvertNALUnitLengths(false)
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
  virtual GMPUnique<GMPVideoEncodedFrame>::Ptr CreateFrame(mp4_demuxer::MP4Sample* aSample);

private:
  const mp4_demuxer::VideoDecoderConfig& mConfig;
  MediaDataDecoderCallbackProxy* mCallback;
  nsCOMPtr<mozIGeckoMediaPluginService> mMPS;
  GMPVideoDecoderProxy* mGMP;
  GMPVideoHost* mHost;
  nsAutoPtr<VideoCallbackAdapter> mAdapter;
  bool mConvertNALUnitLengths;
};

} // namespace mozilla

#endif // GMPVideoDecoder_h_
