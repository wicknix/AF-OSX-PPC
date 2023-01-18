/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef __MiniMP3Decoder_h__
#define __MiniMP3Decoder_h__

#include "MediaDecoder.h"

namespace mozilla {

class MiniMP3Decoder : public MediaDecoder
{
public:
  MiniMP3Decoder();

  virtual MediaDecoder* Clone() override;
  virtual MediaDecoderStateMachine* CreateStateMachine() override;

};

}

#endif // __MiniMP3Decoder_h__
