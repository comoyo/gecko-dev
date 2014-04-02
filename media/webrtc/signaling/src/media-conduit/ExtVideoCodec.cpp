/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "ExtVideoCodec.h"

#ifdef WEBRTC_GONK
#include "WebrtcOMXH264VideoCodec.h"
#endif

namespace mozilla {

VideoEncoder*
ExtVideoCodec::CreateEncoder(CodecType aCodecType)
{
  if (aCodecType == CODEC_H264) {
    return new WebrtcOMXH264VideoEncoder();
  }
  return nullptr;
}

VideoDecoder*
ExtVideoCodec::CreateDecoder(CodecType aCodecType) {
  if (aCodecType == CODEC_H264) {
    return new WebrtcOMXH264VideoDecoder();
  }
  return nullptr;
}

}
