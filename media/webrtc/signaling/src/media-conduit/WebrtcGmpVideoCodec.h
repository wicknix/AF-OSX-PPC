/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

// Class templates copied from WebRTC:
/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


#ifndef WEBRTCGMPVIDEOCODEC_H_
#define WEBRTCGMPVIDEOCODEC_H_

#include <iostream>
#include <queue>

#include "nsThreadUtils.h"
#include "mozilla/Mutex.h"

#include "mozIGeckoMediaPluginService.h"
#include "MediaConduitInterface.h"
#include "AudioConduit.h"
#include "VideoConduit.h"
#include "webrtc/modules/video_coding/codecs/interface/video_codec_interface.h"

#include "gmp-video-host.h"
#include "GMPVideoDecoderProxy.h"
#include "GMPVideoEncoderProxy.h"

namespace mozilla {

class WebrtcGmpVideoEncoder : public WebrtcVideoEncoder,
                              public GMPVideoEncoderCallbackProxy
{
public:
  WebrtcGmpVideoEncoder();
  virtual ~WebrtcGmpVideoEncoder();

  // Implement VideoEncoder interface.
  virtual const uint64_t PluginID() override
  {
    return mGMP ? mGMP->ParentID() : mCachedPluginId;
  }

  virtual void Terminated() override;

  virtual int32_t InitEncode(const webrtc::VideoCodec* aCodecSettings,
                             int32_t aNumberOfCores,
                             uint32_t aMaxPayloadSize) override;

  virtual int32_t Encode(const webrtc::I420VideoFrame& aInputImage,
                         const webrtc::CodecSpecificInfo* aCodecSpecificInfo,
                         const std::vector<webrtc::VideoFrameType>* aFrameTypes) override;

  virtual int32_t RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* aCallback) override;

  virtual int32_t Release() override;

  virtual int32_t SetChannelParameters(uint32_t aPacketLoss,
                                       int aRTT) override;

  virtual int32_t SetRates(uint32_t aNewBitRate,
                           uint32_t aFrameRate) override;

  // GMPVideoEncoderCallback virtual functions.
  virtual void Encoded(GMPVideoEncodedFrame* aEncodedFrame,
                       const nsTArray<uint8_t>& aCodecSpecificInfo) override;

  virtual void Error(GMPErr aError) override {
  }

private:
  virtual int32_t InitEncode_g(const webrtc::VideoCodec* aCodecSettings,
                               int32_t aNumberOfCores,
                               uint32_t aMaxPayloadSize);

  virtual int32_t Encode_g(const webrtc::I420VideoFrame* aInputImage,
                           const webrtc::CodecSpecificInfo* aCodecSpecificInfo,
                           const std::vector<webrtc::VideoFrameType>* aFrameTypes);

  virtual int32_t SetRates_g(uint32_t aNewBitRate,
                             uint32_t aFrameRate);

  nsCOMPtr<mozIGeckoMediaPluginService> mMPS;
  nsCOMPtr<nsIThread> mGMPThread;
  GMPVideoEncoderProxy* mGMP;
  GMPVideoHost* mHost;
  GMPVideoCodec mCodecParams;
  uint32_t mMaxPayloadSize;
  webrtc::EncodedImageCallback* mCallback;
  uint64_t mCachedPluginId;
};


class WebrtcGmpVideoDecoder : public WebrtcVideoDecoder,
                              public GMPVideoDecoderCallbackProxy
{
public:
  WebrtcGmpVideoDecoder();
  virtual ~WebrtcGmpVideoDecoder();

  // Implement VideoDecoder interface.
  virtual const uint64_t PluginID() override
  {
    return mGMP ? mGMP->ParentID() : mCachedPluginId;
  }

  virtual void Terminated() override;

  virtual int32_t InitDecode(const webrtc::VideoCodec* aCodecSettings,
                             int32_t aNumberOfCores) override;
  virtual int32_t Decode(const webrtc::EncodedImage& aInputImage,
                         bool aMissingFrames,
                         const webrtc::RTPFragmentationHeader* aFragmentation,
                         const webrtc::CodecSpecificInfo* aCodecSpecificInfo = nullptr,
                         int64_t aRenderTimeMs = -1) override;
  virtual int32_t RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* aCallback) override;

  virtual int32_t Release() override;

  virtual int32_t Reset() override;

  virtual void Decoded(GMPVideoi420Frame* aDecodedFrame) override;

  virtual void ReceivedDecodedReferenceFrame(const uint64_t aPictureId) override {
    MOZ_CRASH();
  }

  virtual void ReceivedDecodedFrame(const uint64_t aPictureId) override {
    MOZ_CRASH();
  }

  virtual void InputDataExhausted() override {
  }

  virtual void DrainComplete() override {
  }

  virtual void ResetComplete() override {
  }

  virtual void Error(GMPErr aError) override {
     mDecoderStatus = aError;
  }

private:
  virtual int32_t InitDecode_g(const webrtc::VideoCodec* aCodecSettings,
                               int32_t aNumberOfCores);

  virtual int32_t Decode_g(const webrtc::EncodedImage& aInputImage,
                           bool aMissingFrames,
                           const webrtc::RTPFragmentationHeader* aFragmentation,
                           const webrtc::CodecSpecificInfo* aCodecSpecificInfo,
                           int64_t aRenderTimeMs);

  nsCOMPtr<mozIGeckoMediaPluginService> mMPS;
  nsCOMPtr<nsIThread> mGMPThread;
  GMPVideoDecoderProxy* mGMP; // Addref is held for us
  GMPVideoHost* mHost;
  webrtc::DecodedImageCallback* mCallback;
  uint64_t mCachedPluginId;
  GMPErr mDecoderStatus;
};

}

#endif
