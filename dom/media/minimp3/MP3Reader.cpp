/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/* Interface to minimp3 (C)2014-5 Cameron Kaiser, all rights reserved
   For TenFourFox */

#include "MP3Reader.h"

#include "nsISeekableStream.h"
#include "MediaDecoder.h"

namespace mozilla {

#ifdef PR_LOGGING
extern PRLogModuleInfo* gMediaDecoderLog;
#define LOGE(...) PR_LOG(gMediaDecoderLog, PR_LOG_ERROR, (__VA_ARGS__))
#define LOGW(...) PR_LOG(gMediaDecoderLog, PR_LOG_WARNING, (__VA_ARGS__))
#define LOGD(...) PR_LOG(gMediaDecoderLog, PR_LOG_DEBUG, (__VA_ARGS__))
#else
#define LOGE(...)
#define LOGW(...)
#define LOGD(...)
#endif

MiniMP3Reader::MiniMP3Reader(AbstractMediaDecoder *aDecoder)
  : MediaDecoderReader(aDecoder)
  , mMP3FrameParser(mDecoder->GetResource()->GetLength())
  , mDuration(0)
  , mPosition(0)
  , mFrameSize(0)
  , mBufferHasBytes(0)
  , mOffset(0)
{
  MOZ_ASSERT(NS_IsMainThread(), "Should be on main thread");
}

MiniMP3Reader::~MiniMP3Reader()
{
  MOZ_ASSERT(NS_IsMainThread(), "Should be on main thread");

  // Amazon seeks to the beginning, which could push audio.
#if DEBUG
  fprintf(stderr, "mp3_close()\n");
#endif
  mp3_close(&mMP3Decoder);
}

/*
 * If we're not at end of stream, read |aNumBytes| from the media resource,
 * put it in mBuffer, and return true.
 * Otherwise, put as much data as is left into mBuffer, set |aNumBytes| to the
 * amount of data we have left, and return false.
 */
nsresult
MiniMP3Reader::Read(uint32_t *aNumBytes)
{
  // Already some bytes to be read.
  if (mBufferHasBytes >= *aNumBytes) return NS_OK;

  MediaResource *resource = mDecoder->GetResource();

  // Loop until we have all the data asked for, or we've reached EOS
  uint32_t totalBytes = mBufferHasBytes;
  uint32_t numBytes;
  do {
    uint32_t bytesWanted = *aNumBytes - totalBytes;
    nsresult rv = resource->Read(mBuffer + totalBytes, bytesWanted, &numBytes);
    totalBytes += numBytes;

    if (NS_FAILED(rv)) {
      *aNumBytes = 0;
      return NS_ERROR_FAILURE;
    }
  } while(totalBytes < *aNumBytes && numBytes);

  *aNumBytes = totalBytes;
  mBufferHasBytes = totalBytes;

  // We will have read some data in the last iteration iff we filled the buffer.
  // XXX Maybe return a better value than NS_ERROR_FAILURE?
  return numBytes ? NS_OK : NS_ERROR_FAILURE;
}

/* Move buffer up so that unused bytes from the last cycle are again
   consumed. aNumBytes is the offset. */
void
MiniMP3Reader::MoveBufferUp(uint32_t aNumBytes)
{
  mBufferHasBytes -= aNumBytes;
  bcopy((void *)(mBuffer + aNumBytes), (void *)mBuffer, mBufferHasBytes);
}

nsresult
MiniMP3Reader::InitDecoder()
{
  int rv = mp3_init(&mMP3Decoder);
#if DEBUG
  fprintf(stderr, "mp3_init(%i)\n", rv);
#endif
  return (rv) ? NS_ERROR_FAILURE : NS_OK; 
}

nsresult
MiniMP3Reader::Init(MediaDecoderReader* aCloneDonor)
{
#if defined(MOZ_SAMPLE_TYPE_FLOAT32)
#else
#error Unknown audio sample type
#endif
   return InitDecoder();
}

void
MiniMP3Reader::MaybeUpdateDuration()
{
  int64_t duration = mMP3FrameParser.GetDuration();
  if (duration < 0) return;
  if (duration != mDuration) {
    LOGD("Updating media duration to %lluus\n", duration);
#if DEBUG
    fprintf(stderr, "Updating media duration to %lluus\n", duration);
#endif
    mDuration = duration;
    ReentrantMonitorAutoEnter mon(mDecoder->GetReentrantMonitor());
    mDecoder->UpdateEstimatedMediaDuration(duration);
/*
    mDecoder->SetMediaSeekable(true);
    mDecoder->SetTransportSeekable(true);
*/
  }
}

/* minimp3 gets upset if we reseek the stream after the frame parser has read
   so we must feed data as we decode samples instead. */
bool
MiniMP3Reader::DecodeAudioData()
{
	return InternalDecodeAudioData();
}

bool
MiniMP3Reader::InternalDecodeAudioData(bool aPushSamples)
{
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");

  if (!mPosition) {
	// We just seeked to the beginning of a track, which could be a
	// mega-frame. We don't do this in Seek(), because some players
	// seek to the beginning as they move to the next track (Amazon).
	// This doesn't happen the first time through because ReadMetadata
	// handles that.
	return (ReadGiantFirstFrame(aPushSamples) == NS_OK) ? true : false;
  } 

  /* Read frames synchronously. ::Read() will get us that many (we hope).
     We no longer have to account for "jumbo frames" so we can read more
     leisurely, possibly entirely from cached data in the buffer. */
  uint32_t numBytes = AUDIO_READ_BYTES;
  nsresult readrv = Read(&numBytes);
  signed short samples[(1152*2)];

  int newframesize = mp3_decode(&mMP3Decoder, mBuffer, numBytes, samples,
	nullptr);

  if (NS_FAILED(readrv) || newframesize < 1) { // end of stream
    mAudioQueue.Finish();
    return false;
  }

  /* Does the parser still want a whack at it? */
  if (mMP3FrameParser.NeedsData()) {
  	mMP3FrameParser.Parse(mBuffer, newframesize, mOffset);
	mOffset += newframesize;
	MaybeUpdateDuration();
  }
  if (aPushSamples) PushSamples(samples);
  MoveBufferUp(newframesize);
  return true;
}

void
MiniMP3Reader::PushSamples(signed short *aSamples)
{
  // The number of frames is the total number of samples, divided by the
  // number of channels. Since this is an array of signed shorts, the
  // number of frames is audio_bytes / 2 / channels.
  MOZ_ASSERT(mMP3Info.audio_bytes > 0 && mMP3Info.audio_bytes <= 1152*2*2,
	"Unpossible audio_bytes setting");
  MOZ_ASSERT(mMP3Info.channels == 1 || mMP3Info.channels == 2,
	"Unexpected channel setting");
  uint32_t cframes = mMP3Info.audio_bytes / 2;
  uint32_t frames = cframes / mMP3Info.channels;

  // Essentially do as the WaveReader does, except we know that we have
  // native endian shorts, so our job is a little simpler.
  nsAutoArrayPtr<AudioDataValue> sampleBuffer(new AudioDataValue[cframes]);
  AudioDataValue *s = sampleBuffer.get();

  if (MOZ_LIKELY((mMP3Info.audio_bytes & 1) == 0)) {
	if (MOZ_LIKELY((mMP3Info.audio_bytes & 3) == 0)) {
		// Unroll the loop even more.
  		for (int i=0; i < cframes; i++) {
			*s++ = aSamples[i++]/32768.0f;
			*s++ = aSamples[i++]/32768.0f;
			*s++ = aSamples[i++]/32768.0f;
			*s++ = aSamples[i]/32768.0f;
		}
	} else {
		// Unroll the loop.
  		for (int i=0; i < cframes; i++) {
			*s++ = aSamples[i++]/32768.0f;
			*s++ = aSamples[i]/32768.0f;
		}
	}
  } else {
	// Don't unroll the loop.
  	for (int i=0; i < cframes; i++) { *s++ = aSamples[i]/32768.0f; }
  }

  // Time in play is frames * sample rate.
  int64_t playTime = frames * USECS_PER_S / mMP3Info.sample_rate;

  mAudioQueue.Push(new AudioData(mDecoder->GetResource()->Tell(),
                                 static_cast<int64_t>(mPosition),
                                 static_cast<int64_t>(playTime),
                                 static_cast<uint32_t>(frames),
                                 sampleBuffer.forget(),
                                 mMP3Info.channels,
				 mMP3Info.sample_rate));

  mPosition += playTime;
}

bool
MiniMP3Reader::DecodeVideoFrame(bool &aKeyframeSkip,
                                 int64_t aTimeThreshold)
{
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");
  return false;
}


bool
MiniMP3Reader::HasAudio()
{
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");
  return (mFrameSize > 0);
}

bool
MiniMP3Reader::HasVideo()
{
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");
  return false;
}

nsresult
MiniMP3Reader::ReadGiantFirstFrame(bool aPushSamples)
{
  mOffset = 0;
  mBufferHasBytes = 0;
  mFrameSize = 0;
  signed short samplebuf[(1152*2)];
  uint32_t bytesRead = 0;
  nsresult rv;

  /* Some files are seen to have a "mega" initial frame (Amazon is the most
     notorious) and we must read them with a larger than normal buffer. */
  {
  	bytesRead = MAXIMUM_ALLOWABLE_FRAME_SIZE;
  	rv = Read(&bytesRead);
  	if (!bytesRead) return NS_ERROR_FAILURE;

	/* It's possible to return failure, but have bytes read, such as in
	   the situation of a very short MP3 file smaller than the mega frame
	   size. Thus, if we have bytes, we process them. */

  	mFrameSize = mp3_decode(&mMP3Decoder, (void *)mBuffer, bytesRead,
		samplebuf, &mMP3Info);
  	mMP3FrameParser.Parse(mBuffer, bytesRead, mOffset);
  	mOffset += bytesRead;
  } //while (!mFrameSize && NS_SUCCEEDED(rv) && bytesRead);

#if DEBUG
fprintf(stderr, "GiantFirstFrame = %i\n", mFrameSize);
#endif
  MoveBufferUp(mFrameSize);
  // Only frob the duration if it wasn't already set, because this gets
  // called when we Seek() to the beginning as well.
  if (mDuration == -1) {
    ReentrantMonitorAutoEnter mon(mDecoder->GetReentrantMonitor());
    mDuration = mMP3FrameParser.GetDuration();
    if (mDuration != -1) {
#if DEBUG
        fprintf(stderr, "Updating media duration to %lluus\n", mDuration);
#endif
        mDecoder->SetMediaDuration(mDuration);
    } else {
#if DEBUG
	fprintf(stderr, "waiting on media duration\n");
#endif
    }
  }
  if (aPushSamples) PushSamples(samplebuf);
  return NS_OK;
}

nsresult
MiniMP3Reader::ReadMetadata(MediaInfo* aInfo,
                             MetadataTags** aTags)
{
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");

  mDuration = -1;
  *aTags = nullptr;
  // This is the initial load, so we always push audio.
  nsresult rv = ReadGiantFirstFrame();
  if (!mMP3FrameParser.IsMP3() || !mFrameSize || mMP3Info.audio_bytes < 0
	|| mMP3Info.audio_bytes > 1152*2*2) {
#if DEBUG
    fprintf(stderr, "failed\n");
#endif
    LOGE("Frame parser failed to parse MP3 stream\n");
    return (rv != NS_OK) ? rv : NS_ERROR_FAILURE;
  }
  aInfo->mAudio.mRate = mMP3Info.sample_rate;
  aInfo->mAudio.mChannels = mMP3Info.channels;
  aInfo->mAudio.mHasAudio = true;
#if DEBUG
  fprintf(stderr, "framesize = %i\n", mFrameSize);
  fprintf(stderr, "sample rate = %i channels = %i audio_bytes = %i\n",
	mMP3Info.sample_rate,
	mMP3Info.channels,
	mMP3Info.audio_bytes);
#endif
  {
    ReentrantMonitorAutoEnter mon(mDecoder->GetReentrantMonitor());
    // Possible to set the duration early in ReadGiantFirstFrame().
    if (mDuration == -1) 
    	mDuration = mMP3FrameParser.GetDuration();
    if (mDuration == -1) {
/*
    	mDecoder->SetMediaSeekable(false);
    	mDecoder->SetTransportSeekable(false);
*/
    } else {
#if DEBUG
    fprintf(stderr, "Updating media duration to %lluus\n", mDuration);
#endif
	mDecoder->SetMediaDuration(mDuration);
    }
  }
  return NS_OK;
}

nsRefPtr<MediaDecoderReader::SeekPromise>
MiniMP3Reader::Seek(int64_t aTime, int64_t aEndTime)
{
#if DEBUG
  fprintf(stderr, "Seek()\n");
#endif
  MOZ_ASSERT(mDecoder->OnDecodeThread(), "Should be on decode thread");

  // Compute and cache frame data, since mMP3Info could be clobbered when
  // we reseek.
  uint32_t cframes = mMP3Info.audio_bytes / 2;
  uint32_t frames = cframes / mMP3Info.channels;
  // Time in play is frames * sample rate.
  int64_t playTime = frames * USECS_PER_S / mMP3Info.sample_rate;

  // Seek to the beginning, which could be a mega frame.
  mDecoder->GetResource()->Seek(nsISeekableStream::NS_SEEK_SET, 0);
  if (NS_FAILED(ResetDecode()) || NS_FAILED(InitDecoder()))
	return SeekPromise::CreateAndReject(NS_ERROR_FAILURE, __func__);
  mPosition = 0;

  if (!aTime) {
	// Easy peasy. DecodeAudioData will call ReadGiantFirstFrame.
	return SeekPromise::CreateAndResolve(aTime, __func__);
  }

  // Seek to an arbitrary position mid-stream. Theory of operation:
  // ReadGiantFirstFrame() reads the first frame, keeping what it doesn't
  // parse in the buffer. Since InternalDecodeAudioData will call that for
  // us, we just call InternalDecodeAudioData over and over, without pushing
  // audio, until we get to the right part in the stream (i.e., accumulated
  // position is at least aTime), and then return the SeekPromise.

  while(mPosition < aTime) {
	if (!InternalDecodeAudioData(false)) {
		return SeekPromise::CreateAndReject(NS_ERROR_FAILURE,
			__func__);
	}
	mPosition += playTime;
  }
  return SeekPromise::CreateAndResolve(aTime, __func__);
}

void
MiniMP3Reader::NotifyDataArrived(const char* aBuffer,
                                  uint32_t aLength,
                                  int64_t aOffset)
{
  MOZ_ASSERT(NS_IsMainThread());
  if (!mMP3FrameParser.NeedsData()) return;
  mMP3FrameParser.Parse(aBuffer, aLength, aOffset);
  MaybeUpdateDuration();
}

bool
MiniMP3Reader::IsMediaSeekable()
{
  // not used
  return true;
}

} // namespace mozilla
