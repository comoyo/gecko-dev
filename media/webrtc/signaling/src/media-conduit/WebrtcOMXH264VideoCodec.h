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

#ifndef WEBRTC_GONK
#pragma error WebrtcOMXH264VideoCodec works only on B2G.
#endif

#ifndef WEBRTC_OMX_H264_CODEC_H_
#define WEBRTC_OMX_H264_CODEC_H_

#include <queue>

#include "nsThreadUtils.h"
#include "mozilla/Mutex.h"

#include "MediaConduitInterface.h"
#include "AudioConduit.h"
#include "VideoConduit.h"
#include "modules/video_coding/codecs/interface/video_codec_interface.h"

namespace android {
  class OMXVideoEncoder;
}

namespace mozilla {

class WebrtcOMX;

struct EncodeConfig
{
uint32_t mWidth;
uint32_t mHeight;
uint32_t mFrameRate;
};

struct EncodedFrame
{
  uint32_t mWidth;
  uint32_t mHeight;
  uint32_t mTimestamp;
};

class WebrtcOMXH264VideoEncoder : public WebrtcVideoEncoder
{
 public:
  WebrtcOMXH264VideoEncoder();

  virtual ~WebrtcOMXH264VideoEncoder();

  // Implement VideoEncoder interface.
  virtual int32_t InitEncode(const webrtc::VideoCodec* codecSettings,
                                   int32_t numberOfCores,
                                   uint32_t maxPayloadSize);

  virtual int32_t Encode(const webrtc::I420VideoFrame& inputImage,
      const webrtc::CodecSpecificInfo* codecSpecificInfo,
      const std::vector<webrtc::VideoFrameType>* frameTypes);

  virtual int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t SetChannelParameters(uint32_t packetLoss,
                                             int rtt);

  virtual int32_t SetRates(uint32_t newBitRate,
                                 uint32_t frameRate);

 private:
  android::OMXVideoEncoder* mOMX;
  webrtc::EncodedImageCallback* mCallback;
  Mutex mMutex;
  std::queue<EncodedFrame> mFrames;
  webrtc::EncodedImage mEncodedImage;
  size_t mMaxPayloadSize;
  bool mOMXConfigured;
  struct EncodeConfig mConfig;
};

class WebrtcOMXH264VideoDecoder : public WebrtcVideoDecoder
{
 public:
  WebrtcOMXH264VideoDecoder();

  virtual ~WebrtcOMXH264VideoDecoder();

  // Implement VideoDecoder interface.
  virtual int32_t InitDecode(const webrtc::VideoCodec* codecSettings,
                                   int32_t numberOfCores);
  virtual int32_t Decode(const webrtc::EncodedImage& inputImage,
                               bool missingFrames,
                               const webrtc::RTPFragmentationHeader* fragmentation,
                               const webrtc::CodecSpecificInfo*
                               codecSpecificInfo = nullptr,
                               int64_t renderTimeMs = -1);
  virtual int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t Reset();

 private:
  WebrtcOMX* mOMX;
  webrtc::DecodedImageCallback* mCallback;
};

}

#endif // WEBRTC_OMX_H264_CODEC_H_
