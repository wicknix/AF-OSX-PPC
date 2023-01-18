/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#if !defined(MediaDecoderReader_h_)
#define MediaDecoderReader_h_

#include "AbstractMediaDecoder.h"
#include "MediaInfo.h"
#include "MediaData.h"
#include "MediaPromise.h"
#include "MediaQueue.h"
#include "AudioCompactor.h"

namespace mozilla {

namespace dom {
class TimeRanges;
}

class MediaDecoderReader;
class SharedDecoderManager;

struct WaitForDataRejectValue {
  enum Reason {
    SHUTDOWN,
    CANCELED
  };

  WaitForDataRejectValue(MediaData::Type aType, Reason aReason)
    :mType(aType), mReason(aReason) {}
  MediaData::Type mType;
  Reason mReason;
};

// Encapsulates the decoding and reading of media data. Reading can either
// synchronous and done on the calling "decode" thread, or asynchronous and
// performed on a background thread, with the result being returned by
// callback. Never hold the decoder monitor when calling into this class.
// Unless otherwise specified, methods and fields of this class can only
// be accessed on the decode task queue.
class MediaDecoderReader {
public:
  enum NotDecodedReason {
    END_OF_STREAM,
    DECODE_ERROR,
    WAITING_FOR_DATA,
    CANCELED
  };

  typedef MediaPromise<nsRefPtr<AudioData>, NotDecodedReason, /* IsExclusive = */ true> AudioDataPromise;
  typedef MediaPromise<nsRefPtr<VideoData>, NotDecodedReason, /* IsExclusive = */ true> VideoDataPromise;
  typedef MediaPromise<int64_t, nsresult, /* IsExclusive = */ true> SeekPromise;

  // Note that, conceptually, WaitForData makes sense in a non-exclusive sense.
  // But in the current architecture it's only ever used exclusively (by MDSM),
  // so we mark it that way to verify our assumptions. If you have a use-case
  // for multiple WaitForData consumers, feel free to flip the exclusivity here.
  typedef MediaPromise<MediaData::Type, WaitForDataRejectValue, /* IsExclusive = */ true> WaitForDataPromise;

  NS_INLINE_DECL_THREADSAFE_REFCOUNTING(MediaDecoderReader)

  // The caller must ensure that Shutdown() is called before aDecoder is
  // destroyed.
  explicit MediaDecoderReader(AbstractMediaDecoder* aDecoder);

  // Initializes the reader, returns NS_OK on success, or NS_ERROR_FAILURE
  // on failure.
  virtual nsresult Init(MediaDecoderReader* aCloneDonor) = 0;

  // True if this reader is waiting media resource allocation
  virtual bool IsWaitingMediaResources() { return false; }
  // True if this reader is waiting for a Content Decryption Module to become
  // available.
  virtual bool IsWaitingOnCDMResource() { return false; }
  // True when this reader need to become dormant state
  virtual bool IsDormantNeeded() { return false; }
  // Release media resources they should be released in dormant state
  // The reader can be made usable again by calling ReadMetadata().
  virtual void ReleaseMediaResources() {};
  virtual void SetSharedDecoderManager(SharedDecoderManager* aManager) {}
  // Breaks reference-counted cycles. Called during shutdown.
  // WARNING: If you override this, you must call the base implementation
  // in your override.
  virtual void BreakCycles();

  // Destroys the decoding state. The reader cannot be made usable again.
  // This is different from ReleaseMediaResources() as it is irreversable,
  // whereas ReleaseMediaResources() is.  Must be called on the decode
  // thread.
  virtual nsRefPtr<ShutdownPromise> Shutdown();

  MediaTaskQueue* EnsureTaskQueue();

  virtual bool OnDecodeThread()
  {
    return !GetTaskQueue() || GetTaskQueue()->IsCurrentThreadIn();
  }

  void SetBorrowedTaskQueue(MediaTaskQueue* aTaskQueue)
  {
    MOZ_ASSERT(!mTaskQueue && aTaskQueue);
    mTaskQueue = aTaskQueue;
    mTaskQueueIsBorrowed = true;
  }

  // Resets all state related to decoding, emptying all buffers etc.
  // Cancels all pending Request*Data() request callbacks, and flushes the
  // decode pipeline. The decoder must not call any of the callbacks for
  // outstanding Request*Data() calls after this is called. Calls to
  // Request*Data() made after this should be processed as usual.
  // Normally this call preceedes a Seek() call, or shutdown.
  // The first samples of every stream produced after a ResetDecode() call
  // *must* be marked as "discontinuities". If it's not, seeking work won't
  // properly!
  virtual nsresult ResetDecode();

  // Requests one audio sample from the reader.
  //
  // The decode should be performed asynchronously, and the promise should
  // be resolved when it is complete. Don't hold the decoder
  // monitor while calling this, as the implementation may try to wait
  // on something that needs the monitor and deadlock.
  virtual nsRefPtr<AudioDataPromise> RequestAudioData();

  // Requests one video sample from the reader.
  //
  // Don't hold the decoder monitor while calling this, as the implementation
  // may try to wait on something that needs the monitor and deadlock.
  // If aSkipToKeyframe is true, the decode should skip ahead to the
  // the next keyframe at or after aTimeThreshold microseconds.
  virtual nsRefPtr<VideoDataPromise>
  RequestVideoData(bool aSkipToNextKeyframe, int64_t aTimeThreshold);

  friend class ReRequestVideoWithSkipTask;
  friend class ReRequestAudioTask;

  // By default, the state machine polls the reader once per second when it's
  // in buffering mode. Some readers support a promise-based mechanism by which
  // they notify the state machine when the data arrives.
  virtual bool IsWaitForDataSupported() { return false; }
  virtual nsRefPtr<WaitForDataPromise> WaitForData(MediaData::Type aType) { MOZ_CRASH(); }

  virtual bool HasAudio() = 0;
  virtual bool HasVideo() = 0;

  // A function that is called before ReadMetadata() call.
  virtual void PreReadMetadata() {};

  // Read header data for all bitstreams in the file. Fills aInfo with
  // the data required to present the media, and optionally fills *aTags
  // with tag metadata from the file.
  // Returns NS_OK on success, or NS_ERROR_FAILURE on failure.
  virtual nsresult ReadMetadata(MediaInfo* aInfo,
                                MetadataTags** aTags) = 0;

  // Fills aInfo with the latest cached data required to present the media,
  // ReadUpdatedMetadata will always be called once ReadMetadata has succeeded.
  virtual void ReadUpdatedMetadata(MediaInfo* aInfo) { };

  // Moves the decode head to aTime microseconds. aEndTime denotes the end
  // time of the media in usecs. This is only needed for OggReader, and should
  // probably be removed somehow.
  virtual nsRefPtr<SeekPromise>
  Seek(int64_t aTime, int64_t aEndTime) = 0;

  // Cancels an ongoing seek, if any. Any previously-requested seek is
  // guaranteeed to be resolved or rejected in finite time, though no
  // guarantees are made about precise nature of the resolve/reject, since the
  // promise might have already dispatched a resolution or an error code before
  // the cancel arrived.
  //
  // Must be called on the decode task queue.
  virtual void CancelSeek() { };

  // Called to move the reader into idle state. When the reader is
  // created it is assumed to be active (i.e. not idle). When the media
  // element is paused and we don't need to decode any more data, the state
  // machine calls SetIdle() to inform the reader that its decoder won't be
  // needed for a while. The reader can use these notifications to enter
  // a low power state when the decoder isn't needed, if desired.
  // This is most useful on mobile.
  // Note: DecodeVideoFrame, DecodeAudioData, ReadMetadata and Seek should
  // activate the decoder if necessary. The state machine only needs to know
  // when to call SetIdle().
  virtual void SetIdle() { }

  // Tell the reader that the data decoded are not for direct playback, so it
  // can accept more files, in particular those which have more channels than
  // available in the audio output.
  void SetIgnoreAudioOutputFormat()
  {
    mIgnoreAudioOutputFormat = true;
  }

  // Populates aBuffered with the time ranges which are buffered. This function
  // is called on the main, decode, and state machine threads.
  //
  // This base implementation in MediaDecoderReader estimates the time ranges
  // buffered by interpolating the cached byte ranges with the duration
  // of the media. Reader subclasses should override this method if they
  // can quickly calculate the buffered ranges more accurately.
  //
  // The primary advantage of this implementation in the reader base class
  // is that it's a fast approximation, which does not perform any I/O.
  //
  // The OggReader relies on this base implementation not performing I/O,
  // since in FirefoxOS we can't do I/O on the main thread, where this is
  // called.
  virtual nsresult GetBuffered(dom::TimeRanges* aBuffered);

  virtual int64_t ComputeStartTime(const VideoData* aVideo, const AudioData* aAudio);

  // The MediaDecoderStateMachine uses various heuristics that assume that
  // raw media data is arriving sequentially from a network channel. This
  // makes sense in the <video src="foo"> case, but not for more advanced use
  // cases like MSE.
  virtual bool UseBufferingHeuristics() { return true; }

  // Returns the number of bytes of memory allocated by structures/frames in
  // the video queue.
  size_t SizeOfVideoQueueInBytes() const;

  // Returns the number of bytes of memory allocated by structures/frames in
  // the audio queue.
  size_t SizeOfAudioQueueInBytes() const;

  virtual size_t SizeOfVideoQueueInFrames();
  virtual size_t SizeOfAudioQueueInFrames();

  // Only used by WebMReader and MediaOmxReader for now, so stub here rather
  // than in every reader than inherits from MediaDecoderReader.
  virtual void NotifyDataArrived(const char* aBuffer, uint32_t aLength, int64_t aOffset) {}
  virtual int64_t GetEvictionOffset(double aTime) { return -1; }

  virtual MediaQueue<AudioData>& AudioQueue() { return mAudioQueue; }
  virtual MediaQueue<VideoData>& VideoQueue() { return mVideoQueue; }

  // Returns a pointer to the decoder.
  AbstractMediaDecoder* GetDecoder() {
    return mDecoder;
  }

  // TODO: DEPRECATED.  This uses synchronous decoding.
  VideoData* DecodeToFirstVideoData();

  MediaInfo GetMediaInfo() { return mInfo; }

  // Indicates if the media is seekable.
  // ReadMetada should be called before calling this method.
  virtual bool IsMediaSeekable() = 0;
  void SetStartTime(int64_t aStartTime);

  MediaTaskQueue* GetTaskQueue() {
    return mTaskQueue;
  }

  // Returns true if the reader implements RequestAudioData()
  // and RequestVideoData() asynchronously, rather than using the
  // implementation in this class to adapt the old synchronous to
  // the newer async model.
  virtual bool IsAsync() const { return false; }

  virtual void DisableHardwareAcceleration() {}

protected:
  virtual ~MediaDecoderReader();

  // Overrides of this function should decodes an unspecified amount of
  // audio data, enqueuing the audio data in mAudioQueue. Returns true
  // when there's more audio to decode, false if the audio is finished,
  // end of file has been reached, or an un-recoverable read error has
  // occured. This function blocks until the decode is complete.
  virtual bool DecodeAudioData() {
    return false;
  }

  // Overrides of this function should read and decodes one video frame.
  // Packets with a timestamp less than aTimeThreshold will be decoded
  // (unless they're not keyframes and aKeyframeSkip is true), but will
  // not be added to the queue. This function blocks until the decode
  // is complete.
  virtual bool DecodeVideoFrame(bool &aKeyframeSkip, int64_t aTimeThreshold) {
    return false;
  }

  // Queue of audio frames. This queue is threadsafe, and is accessed from
  // the audio, decoder, state machine, and main threads.
  MediaQueue<AudioData> mAudioQueue;

  // Queue of video frames. This queue is threadsafe, and is accessed from
  // the decoder, state machine, and main threads.
  MediaQueue<VideoData> mVideoQueue;

  // An adapter to the audio queue which first copies data to buffers with
  // minimal allocation slop and then pushes them to the queue.  This is
  // useful for decoders working with formats that give awkward numbers of
  // frames such as mp3.
  AudioCompactor mAudioCompactor;

  // Reference to the owning decoder object.
  AbstractMediaDecoder* mDecoder;

  // Stores presentation info required for playback.
  MediaInfo mInfo;

  // Whether we should accept media that we know we can't play
  // directly, because they have a number of channel higher than
  // what we support.
  bool mIgnoreAudioOutputFormat;

  // The start time of the media, in microseconds. This is the presentation
  // time of the first frame decoded from the media. This is initialized to -1,
  // and then set to a value >= by MediaDecoderStateMachine::SetStartTime(),
  // after which point it never changes.
  int64_t mStartTime;

  // This is a quick-and-dirty way for DecodeAudioData implementations to
  // communicate the presence of a decoding error to RequestAudioData. We should
  // replace this with a promise-y mechanism as we make this stuff properly
  // async.
  bool mHitAudioDecodeError;
  bool mShutdown;

private:
  // Promises used only for the base-class (sync->async adapter) implementation
  // of Request{Audio,Video}Data.
  MediaPromiseHolder<AudioDataPromise> mBaseAudioPromise;
  MediaPromiseHolder<VideoDataPromise> mBaseVideoPromise;

  nsRefPtr<MediaTaskQueue> mTaskQueue;
  bool mTaskQueueIsBorrowed;

  // Flags whether a the next audio/video sample comes after a "gap" or
  // "discontinuity" in the stream. For example after a seek.
  bool mAudioDiscontinuity;
  bool mVideoDiscontinuity;
};

} // namespace mozilla

#endif
