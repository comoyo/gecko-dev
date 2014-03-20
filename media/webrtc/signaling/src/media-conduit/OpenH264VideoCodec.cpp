/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "WebrtcOpenH264VideoCodec.h"
#include "OpenH264VideoCodec.h"

namespace mozilla {

VideoEncoder* OpenH264VideoCodec::CreateEncoder() {
  WebrtcOpenH264VideoEncoder *enc =
      new WebrtcOpenH264VideoEncoder();
  return enc;
}

VideoDecoder* OpenH264VideoCodec::CreateDecoder() {
  WebrtcOpenH264VideoDecoder *dec =
      new WebrtcOpenH264VideoDecoder();
  return dec;
}

}
