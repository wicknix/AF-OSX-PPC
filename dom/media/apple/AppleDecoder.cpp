/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "AppleDecoder.h"
#include "AppleMP3Reader.h"

#include "MediaDecoderStateMachine.h"

namespace mozilla {

AppleDecoder::AppleDecoder()
  : MediaDecoder()
{
}

MediaDecoder *
AppleDecoder::Clone()
{
  return new AppleDecoder();
}

MediaDecoderStateMachine *
AppleDecoder::CreateStateMachine()
{
#if(0)
  // TODO MP4
  return new MediaDecoderStateMachine(this, new AppleMP3Reader(this));
#else
  NS_WARNING("MP3 not supported yet in this port of Arctic Fox");
  return nullptr;
#endif
}

} // namespace mozilla
