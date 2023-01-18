/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __MiniMP3Reader_h__
#define __MiniMP3Reader_h__

#include "MediaDecoderReader.h"
#include "MP3FrameParser.h"
#include "VideoUtils.h"

/* relevant bits from minimp3 */

#define MP3_FRAME_SIZE 1152
#define MP3_MAX_CODED_FRAME_SIZE 1792
#define MP3_MAX_CHANNELS 2
#define MAXIMUM_ALLOWABLE_FRAME_SIZE 524288
#define SBLIMIT 32

#define MP3_STEREO  0
#define MP3_JSTEREO 1
#define MP3_DUAL    2
#define MP3_MONO    3

#define HEADER_SIZE 4
#define BACKSTEP_SIZE 512
#define EXTRABYTES 24

#define VLC_TYPE int16_t

#define MP3_MAX_SAMPLES_PER_FRAME (1152*2)
#define TABLE_4_3_SIZE (8191 + 16)*4

typedef struct _bitstream {
    const uint8_t *buffer, *buffer_end;
    int index;
    int size_in_bits;
} bitstream_t;

typedef struct _mp3_context {
    uint8_t last_buf[2*BACKSTEP_SIZE + EXTRABYTES];
    int last_buf_size;
    int frame_size;
    uint32_t free_format_next_header;
    int error_protection;
    int sample_rate;
    int sample_rate_index;
    int bit_rate;
    bitstream_t gb;
    bitstream_t in_gb;
    int nb_channels;
    int mode;
    int mode_ext;
    int lsf;
    int16_t synth_buf[MP3_MAX_CHANNELS][512 * 2];
    int synth_buf_offset[MP3_MAX_CHANNELS];
    int32_t sb_samples[MP3_MAX_CHANNELS][36][SBLIMIT];
    int32_t mdct_buf[MP3_MAX_CHANNELS][SBLIMIT * 18];
    int dither_state;
} mp3_context_t;

typedef void* mp3_decoder_t;

typedef struct _mp3_info {
    int sample_rate;
    int channels;
    int audio_bytes;  // generated amount of audio per frame
} mp3_info_t;

extern "C" int mp3_init(mp3_context_t *dec);
extern "C" int mp3_close(mp3_context_t *dec);
extern "C" int mp3_decode(mp3_context_t *dec, void *buf, int bytes,
	signed short *out, mp3_info_t *info);

/* interface */

#define AUDIO_READ_BYTES 4096

namespace mozilla {

class MiniMP3Reader : public MediaDecoderReader
{
public:
  MiniMP3Reader(AbstractMediaDecoder *aDecoder);
  virtual ~MiniMP3Reader() override;

  virtual nsresult Init(MediaDecoderReader* aCloneDonor) override;

  nsresult PushDataToDemuxer();

  virtual bool DecodeAudioData() override;
  virtual bool DecodeVideoFrame(bool &aKeyframeSkip,
                                int64_t aTimeThreshold) override;

  virtual bool HasAudio() override;
  virtual bool HasVideo() override;

  virtual nsresult ReadMetadata(MediaInfo* aInfo,
                                MetadataTags** aTags) override;

  virtual nsRefPtr<SeekPromise>
  Seek(int64_t aTime, int64_t aEndTime) override;

  virtual void NotifyDataArrived(const char* aBuffer,
                                 uint32_t aLength,
                                 int64_t aOffset) override;

  virtual bool IsMediaSeekable() override;

private:
  nsresult InitDecoder();
  nsresult Read(uint32_t *aNumBytes);
  nsresult ReadGiantFirstFrame(bool aPushSamples = true);
  void MoveBufferUp(uint32_t aNumBytes);
  void PushSamples(signed short *aSamples);
  void MaybeUpdateDuration();
  bool InternalDecodeAudioData(bool aPushSamples = true);

  MP3FrameParser mMP3FrameParser;
  mp3_info_t mMP3Info;
  mp3_context_t mMP3Decoder;

  uint64_t mPosition;
  uint32_t mOffset;
  int64_t mDuration;
  int mFrameSize;
  char mBuffer[MAXIMUM_ALLOWABLE_FRAME_SIZE];
  uint32_t mBufferHasBytes;
};

} // namespace mozilla

#endif // __MiniMP3Reader_h__
