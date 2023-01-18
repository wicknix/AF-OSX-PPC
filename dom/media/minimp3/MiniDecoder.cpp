/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MiniDecoder.h"
#include "MP3Reader.h"

#include "MediaDecoderStateMachine.h"

namespace mozilla {

MiniMP3Decoder::MiniMP3Decoder()
  : MediaDecoder()
{
}

MediaDecoder *
MiniMP3Decoder::Clone()
{
  return new MiniMP3Decoder();
}

MediaDecoderStateMachine *
MiniMP3Decoder::CreateStateMachine()
{
  return new MediaDecoderStateMachine(this, new MiniMP3Reader(this));
}

} // namespace mozilla
