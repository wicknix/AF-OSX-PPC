/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "IntelWebMVideoDecoder.h"

#include "gfx2DGlue.h"
#include "Layers.h"
#include "MediaResource.h"
#include "MediaTaskQueue.h"
#include "mozilla/dom/HTMLMediaElement.h"
#include "nsError.h"
#include "SharedThreadPool.h"
#include "VorbisUtils.h"
#include "nestegg/nestegg.h"

#define VPX_DONT_DEFINE_STDINT_TYPES
#include "vpx/vp8dx.h"
#include "vpx/vpx_decoder.h"

#undef LOG
#ifdef PR_LOGGING
PRLogModuleInfo* GetDemuxerLog();
#define LOG(...) PR_LOG(GetDemuxerLog(), PR_LOG_DEBUG, (__VA_ARGS__))
#else
#define LOG(...)
#endif

using namespace mp4_demuxer;

namespace mozilla {

using layers::Image;
using layers::LayerManager;
using layers::LayersBackend;

class VP8Sample : public MP4Sample
{
public:
  VP8Sample(int64_t aTimestamp,
            int64_t aDuration,
            int64_t aByteOffset,
            uint8_t* aData,
            size_t aSize,
            bool aSyncPoint)
  {
    decode_timestamp = -1;
    composition_timestamp = aTimestamp;
    duration = aDuration;
    byte_offset = aByteOffset;
    is_sync_point = aSyncPoint;

    data =  new uint8_t[aSize];
    size = aSize;
    memmove(data, aData, size);
  }

  virtual ~VP8Sample()
  {
    delete data;
  }
};

IntelWebMVideoDecoder::IntelWebMVideoDecoder(WebMReader* aReader)
  : WebMVideoDecoder()
  , mReader(aReader)
  , mMonitor("IntelWebMVideoDecoder")
  , mNumSamplesInput(0)
  , mNumSamplesOutput(0)
  , mDecodeAhead(2)
  , mInputExhausted(false)
  , mDrainComplete(false)
  , mError(false)
  , mEOS(false)
  , mIsFlushing(false)
{
  MOZ_COUNT_CTOR(IntelWebMVideoDecoder);
}

IntelWebMVideoDecoder::~IntelWebMVideoDecoder()
{
  MOZ_COUNT_DTOR(IntelWebMVideoDecoder);
  Shutdown();
}

void
IntelWebMVideoDecoder::Shutdown()
{
  if (mMediaDataDecoder) {
    Flush();
    mMediaDataDecoder->Shutdown();
    mMediaDataDecoder = nullptr;
  }

  mTaskQueue = nullptr;

  mQueuedVideoSample = nullptr;
  mReader = nullptr;
}

/* static */
WebMVideoDecoder*
IntelWebMVideoDecoder::Create(WebMReader* aReader)
{
  nsAutoPtr<IntelWebMVideoDecoder> decoder(new IntelWebMVideoDecoder(aReader));

  decoder->mTaskQueue = aReader->GetVideoTaskQueue();
  NS_ENSURE_TRUE(decoder->mTaskQueue, nullptr);

  return decoder.forget();
}

bool
IntelWebMVideoDecoder::IsSupportedVideoMimeType(const nsACString& aMimeType)
{
  return (aMimeType.EqualsLiteral("video/webm; codecs=vp8") ||
          aMimeType.EqualsLiteral("video/webm; codecs=vp9")) &&
         mPlatform->SupportsVideoMimeType(aMimeType);
}

nsresult
IntelWebMVideoDecoder::Init(unsigned int aWidth, unsigned int aHeight)
{
  mPlatform = PlatformDecoderModule::Create();
  if (!mPlatform) {
    return NS_ERROR_FAILURE;
  }

  mDecoderConfig = new VideoDecoderConfig();
  mDecoderConfig->duration = 0;
  mDecoderConfig->display_width = aWidth;
  mDecoderConfig->display_height = aHeight;

  switch (mReader->GetVideoCodec()) {
  case NESTEGG_CODEC_VP8:
    mDecoderConfig->mime_type = "video/webm; codecs=vp8";
    break;
  case NESTEGG_CODEC_VP9:
    mDecoderConfig->mime_type = "video/webm; codecs=vp9";
    break;
  default:
    return NS_ERROR_FAILURE;
  }

  const VideoDecoderConfig& video = *mDecoderConfig;
  if (!IsSupportedVideoMimeType(video.mime_type)) {
    return NS_ERROR_FAILURE;
  }
  mMediaDataDecoder = mPlatform->CreateVideoDecoder(video,
                                                    mReader->GetLayersBackendType(),
                                                    mReader->GetDecoder()->GetImageContainer(),
                                                    mTaskQueue,
                                                    this);
  if (!mMediaDataDecoder) {
    return NS_ERROR_FAILURE;
  }
  nsresult rv = mMediaDataDecoder->Init();
  NS_ENSURE_SUCCESS(rv, rv);
  return NS_OK;
}

bool
IntelWebMVideoDecoder::Demux(nsAutoPtr<VP8Sample>& aSample, bool* aEOS)
{
  nsAutoRef<NesteggPacketHolder> holder(mReader->NextPacket(WebMReader::VIDEO));
  if (!holder) {
    return false;
  }

  nestegg_packet* packet = holder->mPacket;
  unsigned int track = 0;
  int r = nestegg_packet_track(packet, &track);
  if (r == -1) {
    return false;
  }

  unsigned int count = 0;
  r = nestegg_packet_count(packet, &count);
  if (r == -1) {
    return false;
  }

  uint64_t tstamp = 0;
  r = nestegg_packet_tstamp(packet, &tstamp);
  if (r == -1) {
    return false;
  }

  // The end time of this frame is the start time of the next frame.  Fetch
  // the timestamp of the next packet for this track.  If we've reached the
  // end of the resource, use the file's duration as the end time of this
  // video frame.
  uint64_t next_tstamp = 0;
  nsAutoRef<NesteggPacketHolder> next_holder(mReader->NextPacket(WebMReader::VIDEO));
  if (next_holder) {
    r = nestegg_packet_tstamp(next_holder->mPacket, &next_tstamp);
    if (r == -1) {
      return false;
    }
    mReader->PushVideoPacket(next_holder.disown());
  } else {
    next_tstamp = tstamp;
    next_tstamp += tstamp - mReader->GetLastVideoFrameTime();
  }
  mReader->SetLastVideoFrameTime(tstamp);

  int64_t tstamp_usecs = tstamp / NS_PER_USEC;
  for (uint32_t i = 0; i < count; ++i) {
    unsigned char* data;
    size_t length;
    r = nestegg_packet_data(packet, i, &data, &length);
    if (r == -1) {
      return false;
    }

    vpx_codec_stream_info_t si;
    memset(&si, 0, sizeof(si));
    si.sz = sizeof(si);
    if (mReader->GetVideoCodec() == NESTEGG_CODEC_VP8) {
      vpx_codec_peek_stream_info(vpx_codec_vp8_dx(), data, length, &si);
    } else if (mReader->GetVideoCodec() == NESTEGG_CODEC_VP9) {
      vpx_codec_peek_stream_info(vpx_codec_vp9_dx(), data, length, &si);
    }

    MOZ_ASSERT(mPlatform && mMediaDataDecoder);

    aSample = new VP8Sample(tstamp_usecs,
                            (next_tstamp/NS_PER_USEC) - tstamp_usecs,
                            0,
                            data,
                            length,
                            si.is_kf);
  }

  return true;
}

bool
IntelWebMVideoDecoder::Decode()
{
  MOZ_ASSERT(mMediaDataDecoder);

  mMonitor.Lock();
  uint64_t prevNumFramesOutput = mNumSamplesOutput;
  while (prevNumFramesOutput == mNumSamplesOutput) {
    mMonitor.AssertCurrentThreadOwns();
    if (mError) {
      // Decode error!
      mMonitor.Unlock();
      return false;
    }
    while (prevNumFramesOutput == mNumSamplesOutput &&
           (mInputExhausted ||
            (mNumSamplesInput - mNumSamplesOutput) < mDecodeAhead) &&
           !mEOS) {
      mMonitor.AssertCurrentThreadOwns();
      mMonitor.Unlock();
      nsAutoPtr<VP8Sample> compressed(PopSample());
      if (!compressed) {
        // EOS, or error. Let the state machine know there are no more
        // frames coming.
        LOG("Draining Video");
        mMonitor.Lock();
        MOZ_ASSERT(!mEOS);
        mEOS = true;
        MOZ_ASSERT(!mDrainComplete);
        mDrainComplete = false;
        mMonitor.Unlock();
        mMediaDataDecoder->Drain();
      } else {
#ifdef LOG_SAMPLE_DECODE
        LOG("PopSample %s time=%lld dur=%lld", TrackTypeToStr(aTrack),
            compressed->composition_timestamp, compressed->duration);
#endif
        mMonitor.Lock();
        mDrainComplete = false;
        mInputExhausted = false;
        mNumSamplesInput++;
        mMonitor.Unlock();
        if (NS_FAILED(mMediaDataDecoder->Input(compressed))) {
          return false;
        }
        // If Input() failed, we let the auto pointer delete |compressed|.
        // Otherwise, we assume the decoder will delete it when it's finished
        // with it.
        compressed.forget();
      }
      mMonitor.Lock();
    }
    mMonitor.AssertCurrentThreadOwns();
    while (!mError &&
           prevNumFramesOutput == mNumSamplesOutput &&
           (!mInputExhausted || mEOS) &&
           !mDrainComplete) {
      mMonitor.Wait();
    }
    if (mError ||
        (mEOS && mDrainComplete)) {
      break;
    }

  }
  mMonitor.AssertCurrentThreadOwns();
  bool rv = !(mEOS || mError);
  mMonitor.Unlock();
  return rv;
}

bool
IntelWebMVideoDecoder::SkipVideoDemuxToNextKeyFrame(int64_t aTimeThreshold, uint32_t& aParsed)
{
  MOZ_ASSERT(mReader->GetDecoder());

  Flush();

  // Loop until we reach the next keyframe after the threshold.
  while (true) {
    nsAutoPtr<VP8Sample> compressed(PopSample());
    if (!compressed) {
      // EOS, or error. Let the state machine know.
      return false;
    }
    aParsed++;
    if (!compressed->is_sync_point ||
        compressed->composition_timestamp < aTimeThreshold) {
      continue;
    }
    mQueuedVideoSample = compressed;
    break;
  }

  return true;
}

bool
IntelWebMVideoDecoder::DecodeVideoFrame(bool& aKeyframeSkip,
                                        int64_t aTimeThreshold)
{
  AbstractMediaDecoder::AutoNotifyDecoded a(mReader->GetDecoder());

  MOZ_ASSERT(mPlatform && mReader->GetDecoder());

  if (aKeyframeSkip) {
    bool ok = SkipVideoDemuxToNextKeyFrame(aTimeThreshold, a.mDropped);
    if (!ok) {
      NS_WARNING("Failed to skip demux up to next keyframe");
      return false;
    }
    a.mParsed = a.mDropped;
    aKeyframeSkip = false;
    nsresult rv = mMediaDataDecoder->Flush();
    NS_ENSURE_SUCCESS(rv, false);
  }

  NS_ASSERTION(mReader->GetDecoder()->OnDecodeThread(), "Should be on decode thread.");
  bool rv = Decode();
  {
    // Report the number of "decoded" frames as the difference in the
    // mNumSamplesOutput field since the last time we were called.
    MonitorAutoLock mon(mMonitor);
    uint64_t delta = mNumSamplesOutput - mLastReportedNumDecodedFrames;
    a.mDecoded = static_cast<uint32_t>(delta);
    mLastReportedNumDecodedFrames = mNumSamplesOutput;
  }
  return rv;
}

VP8Sample*
IntelWebMVideoDecoder::PopSample()
{
  VP8Sample* sample = nullptr;
  if (mQueuedVideoSample) {
    return mQueuedVideoSample.forget();
  }
  while (mSampleQueue.empty()) {
    nsAutoPtr<VP8Sample> sample;
    bool eos = false;
    bool ok = Demux(sample, &eos);
    if (!ok || eos) {
      MOZ_ASSERT(!sample);
      return nullptr;
    }
    MOZ_ASSERT(sample);
    mSampleQueue.push_back(sample.forget());
  }

  MOZ_ASSERT(!mSampleQueue.empty());
  sample = mSampleQueue.front();
  mSampleQueue.pop_front();
  return sample;
}

void
IntelWebMVideoDecoder::Output(MediaData* aSample)
{
#ifdef LOG_SAMPLE_DECODE
  LOG("Decoded video sample time=%lld dur=%lld",
      aSample->mTime, aSample->mDuration);
#endif

  // Don't accept output while we're flushing.
  MonitorAutoLock mon(mMonitor);
  if (mIsFlushing) {
    mon.NotifyAll();
    return;
  }

  MOZ_ASSERT(aSample->mType == MediaData::VIDEO_DATA);
  mReader->VideoQueue().Push(static_cast<VideoData*>(aSample));

  mNumSamplesOutput++;
  mon.NotifyAll();
}

void
IntelWebMVideoDecoder::DrainComplete()
{
  MonitorAutoLock mon(mMonitor);
  mDrainComplete = true;
  mon.NotifyAll();
}

void
IntelWebMVideoDecoder::InputExhausted()
{
  MonitorAutoLock mon(mMonitor);
  mInputExhausted = true;
  mon.NotifyAll();
}

void
IntelWebMVideoDecoder::Error()
{
  MonitorAutoLock mon(mMonitor);
  mError = true;
  mon.NotifyAll();
}

nsresult
IntelWebMVideoDecoder::Flush()
{
  if (!mReader->GetDecoder()) {
    return NS_ERROR_FAILURE;
  }
  // Purge the current decoder's state.
  // Set a flag so that we ignore all output while we call
  // MediaDataDecoder::Flush().
  {
    MonitorAutoLock mon(mMonitor);
    mIsFlushing = true;
    mDrainComplete = false;
    mEOS = false;
  }
  mMediaDataDecoder->Flush();
  {
    MonitorAutoLock mon(mMonitor);
    mIsFlushing = false;
  }
  return NS_OK;
}

} // namespace mozilla
