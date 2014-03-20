/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

// Class templates copied from WebRTC:
/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


#include <queue>

#include "nsThreadUtils.h"
#include "mozilla/Mutex.h"

#include "MediaConduitInterface.h"
#include "AudioConduit.h"
#include "VideoConduit.h"
#include "modules/video_coding/codecs/interface/video_codec_interface.h"

namespace mozilla {

struct EncodedFrame {
  uint32_t width_;
  uint32_t height_;
  uint8_t value_;
  uint32_t timestamp_;
};

class WebrtcFakeVideoEncoder : public WebrtcVideoEncoder {
 public:
  WebrtcFakeVideoEncoder();

  virtual ~WebrtcFakeVideoEncoder() {
  }

  // Implement VideoEncoder interface.
  virtual int32_t InitEncode(const webrtc::VideoCodec* codecSettings,
                                   int32_t numberOfCores,
                                   uint32_t maxPayloadSize);

  virtual int32_t Encode(const webrtc::I420VideoFrame& inputImage,
      const webrtc::CodecSpecificInfo* codecSpecificInfo,
      const std::vector<webrtc::VideoFrameType>* frame_types);

  virtual int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t SetChannelParameters(uint32_t packetLoss,
                                             int rtt);

  virtual int32_t SetRates(uint32_t newBitRate,
                                 uint32_t frameRate);

 private:
  void EmitFrames();
  void EmitFrame(EncodedFrame *frame);

  nsCOMPtr<nsIThread> thread_;
  std::queue<EncodedFrame> frames_;
  uint32_t max_payload_size_;
  uint32_t timestamp_;
  webrtc::EncodedImageCallback* callback_;
  mozilla::Mutex mutex_;
};


class WebrtcFakeVideoDecoder : public WebrtcVideoDecoder {
 public:
  WebrtcFakeVideoDecoder();

  virtual ~WebrtcFakeVideoDecoder() {
  }

  // Implement VideoDecoder interface.
  virtual int32_t InitDecode(const webrtc::VideoCodec* codecSettings,
                                   int32_t numberOfCores);
  virtual int32_t Decode(const webrtc::EncodedImage& inputImage,
                               bool missingFrames,
                               const webrtc::RTPFragmentationHeader* fragmentation,
                               const webrtc::CodecSpecificInfo*
                               codecSpecificInfo = NULL,
                               int64_t renderTimeMs = -1);
  virtual int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t Reset();

 private:
  void RunCallback();

  nsCOMPtr<nsIThread> thread_;
  webrtc::DecodedImageCallback* callback_;
  webrtc::I420VideoFrame decoded_image_;
  mozilla::Mutex mutex_;
};

}
