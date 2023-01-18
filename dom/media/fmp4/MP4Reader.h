/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(MP4Reader_h_)
#define MP4Reader_h_

#include "MediaDecoderReader.h"
#include "nsAutoPtr.h"
#include "PlatformDecoderModule.h"
#include "mp4_demuxer/mp4_demuxer.h"
#include "MediaTaskQueue.h"

#include <deque>
#include "mozilla/Monitor.h"

namespace mozilla {

namespace dom {
class TimeRanges;
}

typedef std::deque<mp4_demuxer::MP4Sample*> MP4SampleQueue;

class MP4Stream;

#if defined(MOZ_GONK_MEDIACODEC) || defined(XP_WIN) || defined(MOZ_APPLEMEDIA) || defined(MOZ_FFMPEG)
#define MP4_READER_DORMANT
#else
#undef MP4_READER_DORMANT
#endif

#if defined(XP_WIN) || defined(MOZ_APPLEMEDIA) || defined(MOZ_FFMPEG)
#define MP4_READER_DORMANT_HEURISTIC
#else
#undef MP4_READER_DORMANT_HEURISTIC
#endif

class MP4Reader final : public MediaDecoderReader
{
  typedef mp4_demuxer::TrackType TrackType;

public:
  explicit MP4Reader(AbstractMediaDecoder* aDecoder);

  virtual ~MP4Reader();

  virtual nsresult Init(MediaDecoderReader* aCloneDonor) override;

  virtual size_t SizeOfVideoQueueInFrames() override;
  virtual size_t SizeOfAudioQueueInFrames() override;

  virtual nsRefPtr<VideoDataPromise>
  RequestVideoData(bool aSkipToNextKeyframe, int64_t aTimeThreshold) override;

  virtual nsRefPtr<AudioDataPromise> RequestAudioData() override;

  virtual bool HasAudio() override;
  virtual bool HasVideo() override;

  // PreReadMetadata() is called by MediaDecoderStateMachine::DecodeMetadata()
  // before checking hardware resource. In Gonk, it requests hardware codec so
  // MediaDecoderStateMachine could go to DORMANT state if the hardware codec is
  // not available.
  virtual void PreReadMetadata() override;
  virtual nsresult ReadMetadata(MediaInfo* aInfo,
                                MetadataTags** aTags) override;

  virtual void ReadUpdatedMetadata(MediaInfo* aInfo) override;

  virtual nsRefPtr<SeekPromise>
  Seek(int64_t aTime, int64_t aEndTime) override;

  virtual bool IsMediaSeekable() override;

  virtual int64_t GetEvictionOffset(double aTime) override;
  virtual void NotifyDataArrived(const char* aBuffer, uint32_t aLength, int64_t aOffset) override;

  virtual nsresult GetBuffered(dom::TimeRanges* aBuffered) override;

  // For Media Resource Management
  virtual void SetIdle() override;
  virtual bool IsWaitingMediaResources() override;
  virtual bool IsDormantNeeded() override;
  virtual void ReleaseMediaResources() override;
  virtual void SetSharedDecoderManager(SharedDecoderManager* aManager)
    override;

  virtual nsresult ResetDecode() override;

  virtual nsRefPtr<ShutdownPromise> Shutdown() override;

  virtual bool IsAsync() const override { return true; }

  virtual void DisableHardwareAcceleration() override;

private:

  bool InitDemuxer();
  void ReturnOutput(MediaData* aData, TrackType aTrack);

  // Sends input to decoder for aTrack, and output to the state machine,
  // if necessary.
  void Update(TrackType aTrack);

  // Enqueues a task to call Update(aTrack) on the decoder task queue.
  // Lock for corresponding track must be held.
  void ScheduleUpdate(TrackType aTrack);

  void ExtractCryptoInitData(nsTArray<uint8_t>& aInitData);

  // Initializes mLayersBackendType if possible.
  void InitLayersBackendType();

  // Blocks until the demuxer produces an sample of specified type.
  // Returns nullptr on error on EOS. Caller must delete sample.
  mp4_demuxer::MP4Sample* PopSample(mp4_demuxer::TrackType aTrack);
  mp4_demuxer::MP4Sample* PopSampleLocked(mp4_demuxer::TrackType aTrack);

  bool SkipVideoDemuxToNextKeyFrame(int64_t aTimeThreshold, uint32_t& parsed);

  // DecoderCallback proxies the MediaDataDecoderCallback calls to these
  // functions.
  void Output(mp4_demuxer::TrackType aType, MediaData* aSample);
  void InputExhausted(mp4_demuxer::TrackType aTrack);
  void Error(mp4_demuxer::TrackType aTrack);
  void Flush(mp4_demuxer::TrackType aTrack);
  void DrainComplete(mp4_demuxer::TrackType aTrack);
  void UpdateIndex();
  bool IsSupportedAudioMimeType(const nsACString& aMimeType);
  bool IsSupportedVideoMimeType(const nsACString& aMimeType);
  void NotifyResourcesStatusChanged();
  void RequestCodecResource();
  bool IsWaitingOnCodecResource();
  virtual bool IsWaitingOnCDMResource() override;

  Microseconds GetNextKeyframeTime();
  bool ShouldSkip(bool aSkipToNextKeyframe, int64_t aTimeThreshold);

  size_t SizeOfQueue(TrackType aTrack);

  nsRefPtr<MP4Stream> mStream;
  nsAutoPtr<mp4_demuxer::MP4Demuxer> mDemuxer;
  nsRefPtr<PlatformDecoderModule> mPlatform;

  class DecoderCallback : public MediaDataDecoderCallback {
  public:
    DecoderCallback(MP4Reader* aReader,
                    mp4_demuxer::TrackType aType)
      : mReader(aReader)
      , mType(aType)
    {
    }
    virtual void Output(MediaData* aSample) override {
      mReader->Output(mType, aSample);
    }
    virtual void InputExhausted() override {
      mReader->InputExhausted(mType);
    }
    virtual void Error() override {
      mReader->Error(mType);
    }
    virtual void DrainComplete() override {
      mReader->DrainComplete(mType);
    }
    virtual void NotifyResourcesStatusChanged() override {
      mReader->NotifyResourcesStatusChanged();
    }
    virtual void ReleaseMediaResources() override {
      mReader->ReleaseMediaResources();
    }
  private:
    MP4Reader* mReader;
    mp4_demuxer::TrackType mType;
  };

  struct DecoderData {
    DecoderData(MediaData::Type aType,
                uint32_t aDecodeAhead)
      : mType(aType)
      , mMonitor(aType == MediaData::AUDIO_DATA ? "MP4 audio decoder data"
                                                : "MP4 video decoder data")
      , mNumSamplesInput(0)
      , mNumSamplesOutput(0)
      , mDecodeAhead(aDecodeAhead)
      , mActive(false)
      , mInputExhausted(false)
      , mError(false)
      , mIsFlushing(false)
      , mUpdateScheduled(false)
      , mDemuxEOS(false)
      , mDrainComplete(false)
      , mDiscontinuity(false)
    {
    }

    // The platform decoder.
    nsRefPtr<MediaDataDecoder> mDecoder;
    // TaskQueue on which decoder can choose to decode.
    // Only non-null up until the decoder is created.
    nsRefPtr<FlushableMediaTaskQueue> mTaskQueue;
    // Callback that receives output and error notifications from the decoder.
    nsAutoPtr<DecoderCallback> mCallback;
    // Decoded samples returned my mDecoder awaiting being returned to
    // state machine upon request.
    nsTArray<nsRefPtr<MediaData> > mOutput;
    // Disambiguate Audio vs Video.
    MediaData::Type mType;

    // These get overriden in the templated concrete class.
    virtual bool HasPromise() = 0;
    virtual void RejectPromise(MediaDecoderReader::NotDecodedReason aReason,
                               const char* aMethodName) = 0;

    // Monitor that protects all non-threadsafe state; the primitives
    // that follow.
    Monitor mMonitor;
    uint64_t mNumSamplesInput;
    uint64_t mNumSamplesOutput;
    uint32_t mDecodeAhead;
    // Whether this stream exists in the media.
    bool mActive;
    bool mInputExhausted;
    bool mError;
    bool mIsFlushing;
    bool mUpdateScheduled;
    bool mDemuxEOS;
    bool mDrainComplete;
    bool mDiscontinuity;
  };

  template<typename PromiseType>
  struct DecoderDataWithPromise : public DecoderData {
    DecoderDataWithPromise(MediaData::Type aType, uint32_t aDecodeAhead) :
      DecoderData(aType, aDecodeAhead)
    {
      mPromise.SetMonitor(&mMonitor);
    }

    MediaPromiseHolder<PromiseType> mPromise;

    bool HasPromise() override { return !mPromise.IsEmpty(); }
    void RejectPromise(MediaDecoderReader::NotDecodedReason aReason,
                       const char* aMethodName) override
    {
      mPromise.Reject(aReason, aMethodName);
    }
  };

  DecoderDataWithPromise<AudioDataPromise> mAudio;
  DecoderDataWithPromise<VideoDataPromise> mVideo;

  // Queued samples extracted by the demuxer, but not yet sent to the platform
  // decoder.
  nsAutoPtr<mp4_demuxer::MP4Sample> mQueuedVideoSample;

  // Returns true when the decoder for this track needs input.
  // aDecoder.mMonitor must be locked.
  bool NeedInput(DecoderData& aDecoder);

  // The last number of decoded output frames that we've reported to
  // MediaDecoder::NotifyDecoded(). We diff the number of output video
  // frames every time that DecodeVideoData() is called, and report the
  // delta there.
  uint64_t mLastReportedNumDecodedFrames;

  DecoderData& GetDecoderData(mp4_demuxer::TrackType aTrack);

  layers::LayersBackend mLayersBackendType;

  nsTArray<nsTArray<uint8_t>> mInitDataEncountered;

  // True if we've read the streams' metadata.
  bool mDemuxerInitialized;

  // True if we've gathered telemetry from an SPS.
  bool mFoundSPSForTelemetry;

  // Synchronized by decoder monitor.
  bool mIsEncrypted;

  bool mIndexReady;
  int64_t mLastSeenEnd;
  Monitor mDemuxerMonitor;
  nsRefPtr<SharedDecoderManager> mSharedDecoderManager;

#if defined(MP4_READER_DORMANT_HEURISTIC)
  const bool mDormantEnabled;
#endif
};

} // namespace mozilla

#endif
