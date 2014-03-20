/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "WebrtcFakeVideoCodec.h"
#include "FakeVideoCodec.h"

namespace mozilla {

VideoEncoder* FakeVideoCodec::CreateEncoder() {
  WebrtcFakeVideoEncoder *enc =
      new WebrtcFakeVideoEncoder();
  return enc;
}

VideoDecoder* FakeVideoCodec::CreateDecoder() {
  WebrtcFakeVideoDecoder *dec =
      new WebrtcFakeVideoDecoder();
  return dec;
}

}
