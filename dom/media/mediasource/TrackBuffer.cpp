/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "TrackBuffer.h"

#include "ContainerParser.h"
#include "MediaData.h"
#include "MediaSourceDecoder.h"
#include "SharedThreadPool.h"
#include "MediaTaskQueue.h"
#include "SourceBufferDecoder.h"
#include "SourceBufferResource.h"
#include "VideoUtils.h"
#include "mozilla/dom/TimeRanges.h"
#include "mozilla/Preferences.h"
#include "nsError.h"
#include "nsIRunnable.h"
#include "nsThreadUtils.h"
#include "prlog.h"

#ifdef PR_LOGGING
extern PRLogModuleInfo* GetMediaSourceLog();

#define MSE_DEBUG(arg, ...) PR_LOG(GetMediaSourceLog(), PR_LOG_DEBUG, ("TrackBuffer(%p:%s)::%s: " arg, this, mType.get(), __func__, ##__VA_ARGS__))
#else
#define MSE_DEBUG(...)
#endif

// Time in seconds to substract from the current time when deciding the
// time point to evict data before in a decoder. This is used to help
// prevent evicting the current playback point.
#define MSE_EVICT_THRESHOLD_TIME 2.0

// Time in microsecond under which a timestamp will be considered to be 0.
#define FUZZ_TIMESTAMP_OFFSET 100000

#define EOS_FUZZ_US 125000

namespace mozilla {

TrackBuffer::TrackBuffer(MediaSourceDecoder* aParentDecoder, const nsACString& aType)
  : mParentDecoder(aParentDecoder)
  , mType(aType)
  , mLastStartTimestamp(0)
  , mLastTimestampOffset(0)
  , mAdjustedTimestamp(0)
  , mShutdown(false)
{
  MOZ_COUNT_CTOR(TrackBuffer);
  mParser = ContainerParser::CreateForMIMEType(aType);
  mTaskQueue = new MediaTaskQueue(GetMediaDecodeThreadPool());
  aParentDecoder->AddTrackBuffer(this);
  mDecoderPerSegment = Preferences::GetBool("media.mediasource.decoder-per-segment", false);
  MSE_DEBUG("TrackBuffer created for parent decoder %p", aParentDecoder);
}

TrackBuffer::~TrackBuffer()
{
  MOZ_COUNT_DTOR(TrackBuffer);
}

class MOZ_STACK_CLASS DecodersToInitialize final {
public:
  explicit DecodersToInitialize(TrackBuffer* aOwner)
    : mOwner(aOwner)
  {
  }

  ~DecodersToInitialize()
  {
    for (size_t i = 0; i < mDecoders.Length(); i++) {
      mOwner->QueueInitializeDecoder(mDecoders[i]);
    }
  }

  bool NewDecoder(int64_t aTimestampOffset)
  {
    nsRefPtr<SourceBufferDecoder> decoder = mOwner->NewDecoder(aTimestampOffset);
    if (!decoder) {
      return false;
    }
    mDecoders.AppendElement(decoder);
    return true;
  }

  size_t Length()
  {
    return mDecoders.Length();
  }

  void AppendElement(SourceBufferDecoder* aDecoder)
  {
    mDecoders.AppendElement(aDecoder);
  }

private:
  TrackBuffer* mOwner;
  nsAutoTArray<nsRefPtr<SourceBufferDecoder>,1> mDecoders;
};

nsRefPtr<ShutdownPromise>
TrackBuffer::Shutdown()
{
  mParentDecoder->GetReentrantMonitor().AssertCurrentThreadIn();
  mShutdown = true;
  mInitializationPromise.RejectIfExists(NS_ERROR_ABORT, __func__);

  MOZ_ASSERT(mShutdownPromise.IsEmpty());
  nsRefPtr<ShutdownPromise> p = mShutdownPromise.Ensure(__func__);

  RefPtr<MediaTaskQueue> queue = mTaskQueue;
  mTaskQueue = nullptr;
  queue->BeginShutdown()
       ->Then(mParentDecoder->GetReader()->GetTaskQueue(), __func__, this,
              &TrackBuffer::ContinueShutdown, &TrackBuffer::ContinueShutdown);

  return p;
}

void
TrackBuffer::ContinueShutdown()
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  if (mDecoders.Length()) {
    mDecoders[0]->GetReader()->Shutdown()
                ->Then(mParentDecoder->GetReader()->GetTaskQueue(), __func__, this,
                       &TrackBuffer::ContinueShutdown, &TrackBuffer::ContinueShutdown);
    mShutdownDecoders.AppendElement(mDecoders[0]);
    mDecoders.RemoveElementAt(0);
    return;
  }

  MOZ_ASSERT(!mCurrentDecoder, "Detach() should have been called");
  mInitializedDecoders.Clear();
  mParentDecoder = nullptr;

  mShutdownPromise.Resolve(true, __func__);
}

nsRefPtr<TrackBufferAppendPromise>
TrackBuffer::AppendData(LargeDataBuffer* aData, int64_t aTimestampOffset)
{
  MOZ_ASSERT(NS_IsMainThread());
  MOZ_ASSERT(mInitializationPromise.IsEmpty());

  DecodersToInitialize decoders(this);
  nsRefPtr<TrackBufferAppendPromise> p = mInitializationPromise.Ensure(__func__);
  bool hadInitData = mParser->HasInitData();
  bool hadCompleteInitData = mParser->HasCompleteInitData();
  nsRefPtr<LargeDataBuffer> oldInit = mParser->InitData();
  bool newInitData = mParser->IsInitSegmentPresent(aData);

  // TODO: Run more of the buffer append algorithm asynchronously.
  if (newInitData) {
    MSE_DEBUG("New initialization segment.");
  } else if (!hadInitData) {
    MSE_DEBUG("Non-init segment appended during initialization.");
    mInitializationPromise.Reject(NS_ERROR_FAILURE, __func__);
    return p;
  }

  int64_t start = 0, end = 0;
  bool gotMedia = mParser->ParseStartAndEndTimestamps(aData, start, end);
  bool gotInit = mParser->HasCompleteInitData();

  if (newInitData) {
    if (!gotInit) {
      // We need a new decoder, but we can't initialize it yet.
      nsRefPtr<SourceBufferDecoder> decoder =
        NewDecoder(aTimestampOffset);
      // The new decoder is stored in mDecoders/mCurrentDecoder, so we
      // don't need to do anything with 'decoder'. It's only a placeholder.
      if (!decoder) {
        mInitializationPromise.Reject(NS_ERROR_FAILURE, __func__);
        return p;
      }
    } else {
      if (!decoders.NewDecoder(aTimestampOffset)) {
        mInitializationPromise.Reject(NS_ERROR_FAILURE, __func__);
        return p;
      }
    }
  } else if (!hadCompleteInitData && gotInit) {
    MOZ_ASSERT(mCurrentDecoder);
    // Queue pending decoder for initialization now that we have a full
    // init segment.
    decoders.AppendElement(mCurrentDecoder);
  }

  if (gotMedia) {
    if (mParser->IsMediaSegmentPresent(aData) && mLastEndTimestamp &&
        (!mParser->TimestampsFuzzyEqual(start, mLastEndTimestamp.value()) ||
         mLastTimestampOffset != aTimestampOffset ||
         mDecoderPerSegment ||
         (mCurrentDecoder && mCurrentDecoder->WasTrimmed()))) {
      MSE_DEBUG("Data last=[%lld, %lld] overlaps [%lld, %lld]",
                mLastStartTimestamp, mLastEndTimestamp.value(), start, end);

      if (!newInitData) {
        // This data is earlier in the timeline than data we have already
        // processed or not continuous, so we must create a new decoder
        // to handle the decoding.
        if (!hadCompleteInitData ||
            !decoders.NewDecoder(aTimestampOffset)) {
          mInitializationPromise.Reject(NS_ERROR_FAILURE, __func__);
          return p;
        }
        MSE_DEBUG("Decoder marked as initialized.");
        AppendDataToCurrentResource(oldInit, 0);
      }
      mLastStartTimestamp = start;
    } else {
      MSE_DEBUG("Segment last=[%lld, %lld] [%lld, %lld]",
                mLastStartTimestamp,
                mLastEndTimestamp ? mLastEndTimestamp.value() : 0, start, end);
    }
    mLastEndTimestamp.reset();
    mLastEndTimestamp.emplace(end);
  }

  if (gotMedia && start != mAdjustedTimestamp &&
      ((start < 0 && -start < FUZZ_TIMESTAMP_OFFSET && start < mAdjustedTimestamp) ||
       (start > 0 && (start < FUZZ_TIMESTAMP_OFFSET || start < mAdjustedTimestamp)))) {
    AdjustDecodersTimestampOffset(mAdjustedTimestamp - start);
    mAdjustedTimestamp = start;
  }

  if (!AppendDataToCurrentResource(aData, end - start)) {
    mInitializationPromise.Reject(NS_ERROR_FAILURE, __func__);
    return p;
  }

  if (decoders.Length()) {
    // We're going to have to wait for the decoder to initialize, the promise
    // will be resolved once initialization completes.
    return p;
  }

  // Tell our reader that we have more data to ensure that playback starts if
  // required when data is appended.
  mParentDecoder->GetReader()->MaybeNotifyHaveData();

  mInitializationPromise.Resolve(gotMedia, __func__);
  return p;
}

bool
TrackBuffer::AppendDataToCurrentResource(LargeDataBuffer* aData, uint32_t aDuration)
{
  MOZ_ASSERT(NS_IsMainThread());
  if (!mCurrentDecoder) {
    return false;
  }

  SourceBufferResource* resource = mCurrentDecoder->GetResource();
  int64_t appendOffset = resource->GetLength();
  resource->AppendData(aData);
  mCurrentDecoder->SetRealMediaDuration(mCurrentDecoder->GetRealMediaDuration() + aDuration);
  // XXX: For future reference: NDA call must run on the main thread.
  mCurrentDecoder->NotifyDataArrived(reinterpret_cast<const char*>(aData->Elements()),
                                     aData->Length(), appendOffset);
  mParentDecoder->NotifyBytesDownloaded();
  mParentDecoder->NotifyTimeRangesChanged();

  return true;
}

class DecoderSorter
{
public:
  bool LessThan(SourceBufferDecoder* aFirst, SourceBufferDecoder* aSecond) const
  {
    nsRefPtr<dom::TimeRanges> first = new dom::TimeRanges();
    aFirst->GetBuffered(first);

    nsRefPtr<dom::TimeRanges> second = new dom::TimeRanges();
    aSecond->GetBuffered(second);

    return first->GetStartTime() < second->GetStartTime();
  }

  bool Equals(SourceBufferDecoder* aFirst, SourceBufferDecoder* aSecond) const
  {
    nsRefPtr<dom::TimeRanges> first = new dom::TimeRanges();
    aFirst->GetBuffered(first);

    nsRefPtr<dom::TimeRanges> second = new dom::TimeRanges();
    aSecond->GetBuffered(second);

    return first->GetStartTime() == second->GetStartTime();
  }
};

bool
TrackBuffer::EvictData(double aPlaybackTime,
                       uint32_t aThreshold,
                       double* aBufferStartTime)
{
  MOZ_ASSERT(NS_IsMainThread());
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  if (!mCurrentDecoder) {
    return false;
  }

  int64_t totalSize = GetSize();

  int64_t toEvict = totalSize - aThreshold;
  if (toEvict <= 0 || mInitializedDecoders.IsEmpty()) {
    return false;
  }

  // Get a list of initialized decoders.
  nsTArray<SourceBufferDecoder*> decoders;
  decoders.AppendElements(mInitializedDecoders);

  // First try to evict data before the current play position, starting
  // with the oldest decoder.
  for (uint32_t i = 0; i < decoders.Length() && toEvict > 0; ++i) {
    nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
    decoders[i]->GetBuffered(buffered);

    MSE_DEBUG("Step1. decoder=%u/%u threshold=%u toEvict=%lld",
              i, decoders.Length(), aThreshold, toEvict);

    // To ensure we don't evict data past the current playback position
    // we apply a threshold of a few seconds back and evict data up to
    // that point.
    if (aPlaybackTime > MSE_EVICT_THRESHOLD_TIME) {
      double time = aPlaybackTime - MSE_EVICT_THRESHOLD_TIME;
      bool isActive = decoders[i] == mCurrentDecoder ||
        mParentDecoder->IsActiveReader(decoders[i]->GetReader());
      if (!isActive && buffered->GetEndTime() < time) {
        // The entire decoder is contained before our current playback time.
        // It can be fully evicted.
        MSE_DEBUG("evicting all bufferedEnd=%f "
                  "aPlaybackTime=%f time=%f, size=%lld",
                  buffered->GetEndTime(), aPlaybackTime, time,
                  decoders[i]->GetResource()->GetSize());
        toEvict -= decoders[i]->GetResource()->EvictAll();
      } else {
        int64_t playbackOffset = decoders[i]->ConvertToByteOffset(time);
        MSE_DEBUG("evicting some bufferedEnd=%f "
                  "aPlaybackTime=%f time=%f, playbackOffset=%lld size=%lld",
                  buffered->GetEndTime(), aPlaybackTime, time,
                  playbackOffset, decoders[i]->GetResource()->GetSize());
        if (playbackOffset > 0) {
          toEvict -= decoders[i]->GetResource()->EvictData(playbackOffset,
                                                           playbackOffset);
        }
      }
    }
  }

  // Evict all data from decoders we've likely already read from.
  for (uint32_t i = 0; i < decoders.Length() && toEvict > 0; ++i) {
    MSE_DEBUG("Step2. decoder=%u/%u threshold=%u toEvict=%lld",
              i, decoders.Length(), aThreshold, toEvict);
    if (mParentDecoder->IsActiveReader(decoders[i]->GetReader())) {
      break;
    }
    if (decoders[i] == mCurrentDecoder) {
      continue;
    }
    nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
    decoders[i]->GetBuffered(buffered);

    // Remove data from older decoders than the current one.
    MSE_DEBUG("evicting all "
              "bufferedStart=%f bufferedEnd=%f aPlaybackTime=%f size=%lld",
              buffered->GetStartTime(), buffered->GetEndTime(),
              aPlaybackTime, decoders[i]->GetResource()->GetSize());
    toEvict -= decoders[i]->GetResource()->EvictAll();
  }

  // Evict all data from future decoders, starting furthest away from
  // current playback position.
  // We will ignore the currently playing decoder and the one playing after that
  // in order to ensure we give enough time to the DASH player to re-buffer
  // as necessary.
  // TODO: This step should be done using RangeRemoval:
  // Something like: RangeRemoval(aPlaybackTime + 60s, End);

  // Find the reader currently being played with.
  SourceBufferDecoder* playingDecoder = nullptr;
  for (uint32_t i = 0; i < decoders.Length() && toEvict > 0; ++i) {
    if (mParentDecoder->IsActiveReader(decoders[i]->GetReader())) {
      playingDecoder = decoders[i];
      break;
    }
  }
  // Find the next decoder we're likely going to play with.
  nsRefPtr<SourceBufferDecoder> nextPlayingDecoder = nullptr;
  if (playingDecoder) {
    nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
    playingDecoder->GetBuffered(buffered);
    nextPlayingDecoder =
      mParentDecoder->SelectDecoder(buffered->GetEndTime() * USECS_PER_S + 1,
                                    EOS_FUZZ_US,
                                    mInitializedDecoders);
  }

  // Sort decoders by their start times.
  decoders.Sort(DecoderSorter());

  for (int32_t i = int32_t(decoders.Length()) - 1; i >= 0 && toEvict > 0; --i) {
    MSE_DEBUG("Step3. decoder=%u/%u threshold=%u toEvict=%lld",
              i, decoders.Length(), aThreshold, toEvict);
    if (decoders[i] == playingDecoder || decoders[i] == nextPlayingDecoder ||
        decoders[i] == mCurrentDecoder) {
      continue;
    }
    nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
    decoders[i]->GetBuffered(buffered);

    MSE_DEBUG("evicting all "
              "bufferedStart=%f bufferedEnd=%f aPlaybackTime=%f size=%lld",
              buffered->GetStartTime(), buffered->GetEndTime(),
              aPlaybackTime, decoders[i]->GetResource()->GetSize());
    toEvict -= decoders[i]->GetResource()->EvictAll();
  }

  RemoveEmptyDecoders(decoders);

  bool evicted = toEvict < (totalSize - aThreshold);
  if (evicted) {
    if (playingDecoder) {
      nsRefPtr<dom::TimeRanges> ranges = new dom::TimeRanges();
      playingDecoder->GetBuffered(ranges);
      *aBufferStartTime = std::max(0.0, ranges->GetStartTime());
    } else {
      // We do not currently have data to play yet.
      // Avoid evicting anymore data to minimize rebuffering time.
      *aBufferStartTime = 0.0;
    }
  }

  return evicted;
}

void
TrackBuffer::RemoveEmptyDecoders(nsTArray<mozilla::SourceBufferDecoder*>& aDecoders)
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  // Remove decoders that have no data in them
  for (uint32_t i = 0; i < aDecoders.Length(); ++i) {
    nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
    aDecoders[i]->GetBuffered(buffered);
    MSE_DEBUG("maybe remove empty decoders=%d "
              "size=%lld start=%f end=%f",
              i, aDecoders[i]->GetResource()->GetSize(),
              buffered->GetStartTime(), buffered->GetEndTime());
    if (aDecoders[i] == mCurrentDecoder ||
        mParentDecoder->IsActiveReader(aDecoders[i]->GetReader())) {
      continue;
    }

    if (aDecoders[i]->GetResource()->GetSize() == 0 ||
        buffered->GetStartTime() < 0.0 ||
        buffered->GetEndTime() < 0.0) {
      MSE_DEBUG("remove empty decoders=%d", i);
      RemoveDecoder(aDecoders[i]);
    }
  }
}

int64_t
TrackBuffer::GetSize()
{
  int64_t totalSize = 0;
  for (uint32_t i = 0; i < mInitializedDecoders.Length(); ++i) {
    totalSize += mInitializedDecoders[i]->GetResource()->GetSize();
  }
  return totalSize;
}

bool
TrackBuffer::HasOnlyIncompleteMedia()
{
  if (!mCurrentDecoder) {
    return false;
  }
  nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
  mCurrentDecoder->GetBuffered(buffered);
  MSE_DEBUG("mCurrentDecoder.size=%lld, start=%f end=%f",
            mCurrentDecoder->GetResource()->GetSize(),
            buffered->GetStartTime(), buffered->GetEndTime());
  return mCurrentDecoder->GetResource()->GetSize() && !buffered->Length();
}

void
TrackBuffer::EvictBefore(double aTime)
{
  MOZ_ASSERT(NS_IsMainThread());
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  for (uint32_t i = 0; i < mInitializedDecoders.Length(); ++i) {
    int64_t endOffset = mInitializedDecoders[i]->ConvertToByteOffset(aTime);
    if (endOffset > 0) {
      MSE_DEBUG("decoder=%u offset=%lld",
                i, endOffset);
      mInitializedDecoders[i]->GetResource()->EvictBefore(endOffset);
    }
  }
}

double
TrackBuffer::Buffered(dom::TimeRanges* aRanges)
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  double highestEndTime = 0;

  for (uint32_t i = 0; i < mInitializedDecoders.Length(); ++i) {
    nsRefPtr<dom::TimeRanges> r = new dom::TimeRanges();
    mInitializedDecoders[i]->GetBuffered(r);
    if (r->Length() > 0) {
      highestEndTime = std::max(highestEndTime, r->GetEndTime());
      aRanges->Union(r, double(mParser->GetRoundingError()) / USECS_PER_S);
    }
  }

  return highestEndTime;
}

already_AddRefed<SourceBufferDecoder>
TrackBuffer::NewDecoder(int64_t aTimestampOffset)
{
  MOZ_ASSERT(NS_IsMainThread());
  MOZ_ASSERT(mParentDecoder);

  DiscardCurrentDecoder();

  nsRefPtr<SourceBufferDecoder> decoder =
    mParentDecoder->CreateSubDecoder(mType, aTimestampOffset - mAdjustedTimestamp);
  if (!decoder) {
    return nullptr;
  }
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  mCurrentDecoder = decoder;
  mDecoders.AppendElement(decoder);

  mLastStartTimestamp = 0;
  mLastEndTimestamp.reset();
  mLastTimestampOffset = aTimestampOffset;

  decoder->SetTaskQueue(mTaskQueue);
  return decoder.forget();
}

bool
TrackBuffer::QueueInitializeDecoder(SourceBufferDecoder* aDecoder)
{
  if (NS_WARN_IF(!mTaskQueue)) {
    mInitializationPromise.RejectIfExists(NS_ERROR_FAILURE, __func__);
    return false;
  }

  RefPtr<nsIRunnable> task =
    NS_NewRunnableMethodWithArg<SourceBufferDecoder*>(this,
                                                      &TrackBuffer::InitializeDecoder,
                                                      aDecoder);
  if (NS_FAILED(mTaskQueue->Dispatch(task))) {
    MSE_DEBUG("failed to enqueue decoder initialization task");
    RemoveDecoder(aDecoder);
    mInitializationPromise.RejectIfExists(NS_ERROR_FAILURE, __func__);
    return false;
  }
  return true;
}

void
TrackBuffer::InitializeDecoder(SourceBufferDecoder* aDecoder)
{
  if (!mParentDecoder) {
    MSE_DEBUG("decoder was shutdown. Aborting initialization.");
    return;
  }
  // ReadMetadata may block the thread waiting on data, so we must be able
  // to leave the monitor while we call it. For the rest of this function
  // we want to hold the monitor though, since we run on a different task queue
  // from the reader and interact heavily with it.
  mParentDecoder->GetReentrantMonitor().AssertNotCurrentThreadIn();
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  if (mCurrentDecoder != aDecoder) {
    MSE_DEBUG("append was cancelled. Aborting initialization.");
    // If we reached this point, the SourceBuffer would have disconnected
    // the promise. So no need to reject it.
    return;
  }

  // We may be shut down at any time by the reader on another thread. So we need
  // to check for this each time we acquire the monitor. If that happens, we
  // need to abort immediately, because the reader has forgotten about us, and
  // important pieces of our state (like mTaskQueue) have also been torn down.
  if (mShutdown) {
    MSE_DEBUG("was shut down. Aborting initialization.");
    RemoveDecoder(aDecoder);
    return;
  }

  MOZ_ASSERT(mTaskQueue->IsCurrentThreadIn());
  MediaDecoderReader* reader = aDecoder->GetReader();
  MSE_DEBUG("Initializing subdecoder %p reader %p",
            aDecoder, reader);

  MediaInfo mi;
  nsAutoPtr<MetadataTags> tags; // TODO: Handle metadata.
  nsresult rv;

  // HACK WARNING:
  // We only reach this point once we know that we have a complete init segment.
  // We don't want the demuxer to do a blocking read as no more data can be
  // appended while this routine is running. Marking the SourceBufferResource
  // as ended will cause any incomplete reads to abort.
  // As this decoder hasn't been initialized yet, the resource isn't yet in use
  // and so it is safe to do so.
  bool wasEnded = aDecoder->GetResource()->IsEnded();
  if (!wasEnded) {
    aDecoder->GetResource()->Ended();
  }
  {
    ReentrantMonitorAutoExit mon(mParentDecoder->GetReentrantMonitor());
    rv = reader->ReadMetadata(&mi, getter_Transfers(tags));
  }
  if (!wasEnded) {
    // Adding an empty buffer will reopen the SourceBufferResource
    nsRefPtr<LargeDataBuffer> emptyBuffer = new LargeDataBuffer;
    aDecoder->GetResource()->AppendData(emptyBuffer);
  }
  // HACK END.

  reader->SetIdle();
  if (mShutdown) {
    MSE_DEBUG("was shut down while reading metadata. Aborting initialization.");
    return;
  }
  if (mCurrentDecoder != aDecoder) {
    MSE_DEBUG("append was cancelled. Aborting initialization.");
    return;
  }

  if (NS_SUCCEEDED(rv) && reader->IsWaitingOnCDMResource()) {
    mWaitingDecoders.AppendElement(aDecoder);
    return;
  }

  aDecoder->SetTaskQueue(nullptr);

  if (NS_FAILED(rv) || (!mi.HasVideo() && !mi.HasAudio())) {
    // XXX: Need to signal error back to owning SourceBuffer.
    MSE_DEBUG("Reader %p failed to initialize rv=%x audio=%d video=%d",
              reader, rv, mi.HasAudio(), mi.HasVideo());
    RemoveDecoder(aDecoder);
    mInitializationPromise.RejectIfExists(NS_ERROR_FAILURE, __func__);
    return;
  }

  if (mi.HasVideo()) {
    MSE_DEBUG("Reader %p video resolution=%dx%d",
              reader, mi.mVideo.mDisplay.width, mi.mVideo.mDisplay.height);
  }
  if (mi.HasAudio()) {
    MSE_DEBUG("Reader %p audio sampleRate=%d channels=%d",
              reader, mi.mAudio.mRate, mi.mAudio.mChannels);
  }

  RefPtr<nsIRunnable> task =
    NS_NewRunnableMethodWithArg<SourceBufferDecoder*>(this,
                                                      &TrackBuffer::CompleteInitializeDecoder,
                                                      aDecoder);
  if (NS_FAILED(NS_DispatchToMainThread(task))) {
    MSE_DEBUG("Failed to enqueue decoder initialization task");
    RemoveDecoder(aDecoder);
    mInitializationPromise.RejectIfExists(NS_ERROR_FAILURE, __func__);
    return;
  }
}

void
TrackBuffer::CompleteInitializeDecoder(SourceBufferDecoder* aDecoder)
{
  if (!mParentDecoder) {
    MSE_DEBUG("was shutdown. Aborting initialization.");
    return;
  }
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  if (mCurrentDecoder != aDecoder) {
    MSE_DEBUG("append was cancelled. Aborting initialization.");
    // If we reached this point, the SourceBuffer would have disconnected
    // the promise. So no need to reject it.
    return;
  }

  if (mShutdown) {
    MSE_DEBUG("was shut down. Aborting initialization.");
    RemoveDecoder(aDecoder);
    return;
  }

  if (!RegisterDecoder(aDecoder)) {
    MSE_DEBUG("Reader %p not activated",
              aDecoder->GetReader());
    RemoveDecoder(aDecoder);
    mInitializationPromise.RejectIfExists(NS_ERROR_FAILURE, __func__);
    return;
  }

  int64_t duration = aDecoder->GetMediaDuration();
  if (!duration) {
    // Treat a duration of 0 as infinity
    duration = -1;
  }
  mParentDecoder->SetInitialDuration(duration);

  // Tell our reader that we have more data to ensure that playback starts if
  // required when data is appended.
  mParentDecoder->GetReader()->MaybeNotifyHaveData();

  MSE_DEBUG("Reader %p activated",
            aDecoder->GetReader());
  mInitializationPromise.ResolveIfExists(aDecoder->GetRealMediaDuration() > 0, __func__);
}

bool
TrackBuffer::ValidateTrackFormats(const MediaInfo& aInfo)
{
  if (mInfo.HasAudio() != aInfo.HasAudio() ||
      mInfo.HasVideo() != aInfo.HasVideo()) {
    MSE_DEBUG("audio/video track mismatch");
    return false;
  }

  // TODO: Support dynamic audio format changes.
  if (mInfo.HasAudio() &&
      (mInfo.mAudio.mRate != aInfo.mAudio.mRate ||
       mInfo.mAudio.mChannels != aInfo.mAudio.mChannels)) {
    MSE_DEBUG("audio format mismatch");
    return false;
  }

  return true;
}

bool
TrackBuffer::RegisterDecoder(SourceBufferDecoder* aDecoder)
{
  mParentDecoder->GetReentrantMonitor().AssertCurrentThreadIn();
  const MediaInfo& info = aDecoder->GetReader()->GetMediaInfo();
  // Initialize the track info since this is the first decoder.
  if (mInitializedDecoders.IsEmpty()) {
    mInfo = info;
    mParentDecoder->OnTrackBufferConfigured(this, mInfo);
  }
  if (!ValidateTrackFormats(info)) {
    MSE_DEBUG("mismatched audio/video tracks");
    return false;
  }
  mInitializedDecoders.AppendElement(aDecoder);
  mParentDecoder->NotifyTimeRangesChanged();
  return true;
}

void
TrackBuffer::DiscardCurrentDecoder()
{
  MOZ_ASSERT(NS_IsMainThread());
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  EndCurrentDecoder();
  mCurrentDecoder = nullptr;
}

void
TrackBuffer::EndCurrentDecoder()
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  if (mCurrentDecoder) {
    mCurrentDecoder->GetResource()->Ended();
  }
}

void
TrackBuffer::Detach()
{
  MOZ_ASSERT(NS_IsMainThread());
  if (mCurrentDecoder) {
    DiscardCurrentDecoder();
  }
}

bool
TrackBuffer::HasInitSegment()
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  return mParser->HasCompleteInitData();
}

bool
TrackBuffer::IsReady()
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  MOZ_ASSERT((mInfo.HasAudio() || mInfo.HasVideo()) || mInitializedDecoders.IsEmpty());
  return mInfo.HasAudio() || mInfo.HasVideo();
}

bool
TrackBuffer::ContainsTime(int64_t aTime, int64_t aTolerance)
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  for (uint32_t i = 0; i < mInitializedDecoders.Length(); ++i) {
    nsRefPtr<dom::TimeRanges> r = new dom::TimeRanges();
    mInitializedDecoders[i]->GetBuffered(r);
    if (r->Find(double(aTime) / USECS_PER_S,
                double(aTolerance) / USECS_PER_S) != dom::TimeRanges::NoIndex) {
      return true;
    }
  }

  return false;
}

void
TrackBuffer::BreakCycles()
{
  MOZ_ASSERT(NS_IsMainThread());

  for (uint32_t i = 0; i < mShutdownDecoders.Length(); ++i) {
    mShutdownDecoders[i]->BreakCycles();
  }
  mShutdownDecoders.Clear();

  // These are cleared in Shutdown()
  MOZ_ASSERT(!mDecoders.Length());
  MOZ_ASSERT(mInitializedDecoders.IsEmpty());
  MOZ_ASSERT(!mParentDecoder);
}

void
TrackBuffer::ResetParserState()
{
  MOZ_ASSERT(NS_IsMainThread());

  if (mParser->HasInitData() && !mParser->HasCompleteInitData()) {
    // We have an incomplete init segment pending. reset current parser and
    // discard the current decoder.
    mParser = ContainerParser::CreateForMIMEType(mType);
    DiscardCurrentDecoder();
  }
}

void
TrackBuffer::AbortAppendData()
{
  DiscardCurrentDecoder();
  // The SourceBuffer would have disconnected its promise.
  // However we must ensure that the MediaPromiseHolder handle all pending
  // promises.
  mInitializationPromise.RejectIfExists(NS_ERROR_ABORT, __func__);
}

const nsTArray<nsRefPtr<SourceBufferDecoder>>&
TrackBuffer::Decoders()
{
  // XXX assert OnDecodeThread
  return mInitializedDecoders;
}

#ifdef MOZ_EME
nsresult
TrackBuffer::SetCDMProxy(CDMProxy* aProxy)
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  for (uint32_t i = 0; i < mDecoders.Length(); ++i) {
    nsresult rv = mDecoders[i]->SetCDMProxy(aProxy);
    NS_ENSURE_SUCCESS(rv, rv);
  }

  for (uint32_t i = 0; i < mWaitingDecoders.Length(); ++i) {
    CDMCaps::AutoLock caps(aProxy->Capabilites());
    caps.CallOnMainThreadWhenCapsAvailable(
      NS_NewRunnableMethodWithArg<SourceBufferDecoder*>(this,
                                                        &TrackBuffer::QueueInitializeDecoder,
                                                        mWaitingDecoders[i]));
  }

  mWaitingDecoders.Clear();

  return NS_OK;
}
#endif

#if defined(DEBUG)
void
TrackBuffer::Dump(const char* aPath)
{
  char path[255];
  PR_snprintf(path, sizeof(path), "%s/trackbuffer-%p", aPath, this);
  PR_MkDir(path, 0700);

  for (uint32_t i = 0; i < mDecoders.Length(); ++i) {
    char buf[255];
    PR_snprintf(buf, sizeof(buf), "%s/reader-%p", path, mDecoders[i]->GetReader());
    PR_MkDir(buf, 0700);

    mDecoders[i]->GetResource()->Dump(buf);
  }
}
#endif

class ReleaseDecoderTask : public nsRunnable {
public:
  explicit ReleaseDecoderTask(SourceBufferDecoder* aDecoder)
    : mDecoder(aDecoder)
  {
  }

  NS_IMETHOD Run() override final {
    mDecoder->GetReader()->BreakCycles();
    mDecoder = nullptr;
    return NS_OK;
  }

private:
  nsRefPtr<SourceBufferDecoder> mDecoder;
};

class DelayedDispatchToMainThread : public nsRunnable {
public:
  DelayedDispatchToMainThread(SourceBufferDecoder* aDecoder, TrackBuffer* aTrackBuffer)
    : mDecoder(aDecoder)
    , mTrackBuffer(aTrackBuffer)
  {
  }

  NS_IMETHOD Run() override final {
    // Shutdown the reader, and remove its reference to the decoder
    // so that it can't accidentally read it after the decoder
    // is destroyed.
    mDecoder->GetReader()->Shutdown();
    RefPtr<nsIRunnable> task = new ReleaseDecoderTask(mDecoder);
    mDecoder = nullptr;
    // task now holds the only ref to the decoder.
    NS_DispatchToMainThread(task);
    return NS_OK;
  }

private:
  nsRefPtr<SourceBufferDecoder> mDecoder;
  nsRefPtr<TrackBuffer> mTrackBuffer;
};

void
TrackBuffer::RemoveDecoder(SourceBufferDecoder* aDecoder)
{
  MSE_DEBUG("TrackBuffer(%p)::RemoveDecoder(%p, %p)", this, aDecoder, aDecoder->GetReader());
  RefPtr<nsIRunnable> task = new DelayedDispatchToMainThread(aDecoder, this);
  {
    ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
    // There should be no other references to the decoder. Assert that
    // we aren't using it in the MediaSourceReader.
    MOZ_ASSERT(!mParentDecoder->IsActiveReader(aDecoder->GetReader()));
    mInitializedDecoders.RemoveElement(aDecoder);
    mDecoders.RemoveElement(aDecoder);
  }
  aDecoder->GetReader()->GetTaskQueue()->Dispatch(task);
}

bool
TrackBuffer::RangeRemoval(media::Microseconds aStart,
                          media::Microseconds aEnd)
{
  MOZ_ASSERT(NS_IsMainThread());
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());

  nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
  media::Microseconds bufferedEnd = media::Microseconds::FromSeconds(Buffered(buffered));
  media::Microseconds bufferedStart = media::Microseconds::FromSeconds(buffered->GetStartTime());

  if (bufferedStart < media::Microseconds(0) || aStart > bufferedEnd || aEnd < bufferedStart) {
    // Nothing to remove.
    return false;
  }

  if (aStart > bufferedStart && aEnd < bufferedEnd) {
    // TODO. We only handle trimming and removal from the start.
    NS_WARNING("RangeRemoval unsupported arguments. "
               "Can only handle trimming (trim left or trim right");
    return false;
  }

  nsTArray<SourceBufferDecoder*> decoders;
  decoders.AppendElements(mInitializedDecoders);

  if (aStart <= bufferedStart && aEnd < bufferedEnd) {
    // Evict data from beginning.
    for (size_t i = 0; i < decoders.Length(); ++i) {
      nsRefPtr<dom::TimeRanges> buffered = new dom::TimeRanges();
      decoders[i]->GetBuffered(buffered);
      if (media::Microseconds::FromSeconds(buffered->GetEndTime()) < aEnd) {
        // Can be fully removed.
        MSE_DEBUG("remove all bufferedEnd=%f size=%lld",
                  buffered->GetEndTime(),
                  decoders[i]->GetResource()->GetSize());
        decoders[i]->GetResource()->EvictAll();
      } else {
        int64_t offset = decoders[i]->ConvertToByteOffset(aEnd.ToSeconds());
        MSE_DEBUG("removing some bufferedEnd=%f offset=%lld size=%lld",
                  buffered->GetEndTime(), offset,
                  decoders[i]->GetResource()->GetSize());
        if (offset > 0) {
          decoders[i]->GetResource()->EvictData(offset, offset);
        }
      }
    }
  } else {
    // Only trimming existing buffers.
    for (size_t i = 0; i < decoders.Length(); ++i) {
      if (aStart <= media::Microseconds::FromSeconds(buffered->GetStartTime())) {
        // It will be entirely emptied, can clear all data.
        decoders[i]->GetResource()->EvictAll();
      } else {
        decoders[i]->Trim(aStart.mValue);
      }
    }
  }

  RemoveEmptyDecoders(decoders);

  return true;
}

void
TrackBuffer::AdjustDecodersTimestampOffset(int32_t aOffset)
{
  ReentrantMonitorAutoEnter mon(mParentDecoder->GetReentrantMonitor());
  for (uint32_t i = 0; i < mDecoders.Length(); i++) {
    mDecoders[i]->SetTimestampOffset(mDecoders[i]->GetTimestampOffset() + aOffset);
  }
}

#undef MSE_DEBUG

} // namespace mozilla
