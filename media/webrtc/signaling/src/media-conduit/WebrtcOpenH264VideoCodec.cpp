/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <memory>

#include "logging.h"
#include "nspr.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"

#include <iostream>

#include <mozilla/Scoped.h>
#include "VideoConduit.h"
#include "AudioConduit.h"

#include "video_engine/include/vie_external_codec.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"
#include "param_svc.h"


#include "runnable_utils.h"

#include "WebrtcOpenH264VideoCodec.h"

namespace mozilla {

MOZ_MTLOG_MODULE("openh264");

struct EncodedFrame {
 public:
  EncodedFrame(uint8_t *buffer, uint32_t size, uint32_t length,
               uint32_t width, uint32_t height, uint32_t timestamp,
               webrtc::VideoFrameType frame_type) :
      image_(buffer, size, length),
      buffer_(buffer) {
    image_._encodedWidth = width;
    image_._encodedHeight = height;
    image_._timeStamp = timestamp;
    image_._frameType = frame_type;
    image_._completeFrame = true;
  }

  static EncodedFrame* Create(const SFrameBSInfo& frame,
                              uint32_t width, uint32_t height,
                              uint32_t timestamp, webrtc::VideoFrameType frame_type) {
    // Buffer up the data.
    uint32_t length = 0;
    std::vector<uint32_t> lengths;

    for (int i=0; i<frame.iLayerNum; ++i) {
      lengths.push_back(0);
      for (int j=0; j<frame.sLayerInfo[i].iNalCount; ++j) {
        lengths[i] += frame.sLayerInfo[i].iNalLengthInByte[j];
        length += frame.sLayerInfo[i].iNalLengthInByte[j];
      }
    }

    ScopedDeleteArray<uint8_t> buffer(new uint8_t[length]);
    uint8_t *tmp = buffer;

    for (int i=0; i<frame.iLayerNum; ++i) {
      // TODO(ekr@rtfm.com): This seems screwy, but I copied it from Cisco.
      memcpy(tmp, frame.sLayerInfo[i].pBsBuf, lengths[i]);
      tmp += lengths[i];
    }

    return new EncodedFrame(buffer.forget(), length, length,
                            width, height, timestamp, frame_type);
  }

  webrtc::EncodedImage& image() { return image_; }

 private:
  webrtc::EncodedImage image_;
  ScopedDeleteArray<uint8_t> buffer_;
};

// Encoder.
WebrtcOpenH264VideoEncoder::WebrtcOpenH264VideoEncoder()
    : callback_(nullptr),
      mutex_("WebrtcOpenH264VideoEncoder") {
  nsIThread* thread;

  nsresult rv = NS_NewNamedThread("encoder-thread", &thread);
  MOZ_ASSERT(NS_SUCCEEDED(rv));

  thread_ = thread;
}

WebrtcOpenH264VideoEncoder::~WebrtcOpenH264VideoEncoder() {
  if (encoder_) {
    DestroySVCEncoder(encoder_);
  }
}

int32_t WebrtcOpenH264VideoEncoder::InitEncode(
    const webrtc::VideoCodec* codecSettings,
    int32_t numberOfCores,
    uint32_t maxPayloadSize) {
  max_payload_size_ = maxPayloadSize;

  int rv = CreateSVCEncoder(&encoder_);
  if (rv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  SEncParamBase param;
  memset(&param, 0, sizeof(param));

  MOZ_MTLOG(ML_INFO, "Initializing encoder at "
	    << codecSettings->width
	    << "x"
	    << codecSettings->height
	    << "@"
	    << static_cast<int>(codecSettings->maxFramerate));

  // Translate parameters.
  param.iPicWidth = codecSettings->width;
  param.iPicHeight = codecSettings->height;
  param.iTargetBitrate = codecSettings->startBitrate * 1000;
  // TODO(ekr@rtfm.com). Scary conversion from unsigned char to float below.
  param.fMaxFrameRate = codecSettings->maxFramerate;
  param.iInputCsp = videoFormatI420;

  rv = encoder_->Initialize(&param);
  if (rv)
    return WEBRTC_VIDEO_CODEC_MEMORY;

  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoEncoder::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  MOZ_MTLOG(ML_DEBUG, "Frame added");

  MOZ_ASSERT(!frame_types->empty());
  if (frame_types->empty())
    return WEBRTC_VIDEO_CODEC_ERROR;
  // TODO(ekr@rtfm.com): Actually handle frame type.

  webrtc::I420VideoFrame* imageCopy = new webrtc::I420VideoFrame();
  imageCopy->CopyFrame(inputImage);

  RUN_ON_THREAD(thread_,
      WrapRunnable(nsRefPtr<WebrtcOpenH264VideoEncoder>(this),
		   &WebrtcOpenH264VideoEncoder::Encode_w,
		   imageCopy, (*frame_types)[0]),
		NS_DISPATCH_NORMAL);

  return WEBRTC_VIDEO_CODEC_OK;
}

void WebrtcOpenH264VideoEncoder::Encode_w(
    webrtc::I420VideoFrame* inputImage,
    webrtc::VideoFrameType frame_type) {
  SFrameBSInfo encoded;
  SSourcePicture src;

  src.iColorFormat = videoFormatI420;
  src.iStride[0] = inputImage->stride(webrtc::kYPlane);
  src.pData[0] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kYPlane)));
  src.iStride[1] = inputImage->stride(webrtc::kUPlane);
  src.pData[1] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kUPlane)));
  src.iStride[2] = inputImage->stride(webrtc::kVPlane);
  src.pData[2] = reinterpret_cast<unsigned char*>(
      const_cast<uint8_t *>(inputImage->buffer(webrtc::kVPlane)));
  src.iStride[3] = 0;
  src.pData[3] = nullptr;
  src.iPicWidth = inputImage->width();
  src.iPicHeight = inputImage->height();

  const SSourcePicture* pics = &src;

  PRIntervalTime t0 = PR_IntervalNow();
  int type = encoder_->EncodeFrame(pics, &encoded);
  PRIntervalTime t1 = PR_IntervalNow();

  MOZ_MTLOG(ML_DEBUG, "Encoding time: " << PR_IntervalToMilliseconds(
      t1 - t0) << "ms");

  // Translate int to enum
  switch (type) {
    case videoFrameTypeIDR:
    case videoFrameTypeI:
    case videoFrameTypeP:
      {
        ScopedDeletePtr<EncodedFrame> encoded_frame(
            EncodedFrame::Create(encoded,
                                 inputImage->width(),
                                 inputImage->height(),
                                 inputImage->timestamp(),
                                 frame_type));
        callback_->Encoded(encoded_frame->image(), NULL, NULL);
      }
      break;
    case videoFrameTypeSkip:
      //can skip the call back since not actual bit stream will be generated
      break;
    case videoFrameTypeIPMixed://this type is currently not suppported
    case videoFrameTypeInvalid:
      MOZ_MTLOG(ML_ERROR, "Couldn't encode frame. Error = " << type);
      break;
    default:
      // The API is defined as returning a type.
      MOZ_CRASH();
      break;
  }
  delete inputImage;
  return;
}

void WebrtcOpenH264VideoEncoder::EmitFrames() {
  MutexAutoLock lock(mutex_);

  while(!frames_.empty()) {
    ScopedDeletePtr<EncodedFrame> frame(frames_.front());
    MOZ_MTLOG(ML_DEBUG, "Emitting frame length=" << frame->image()._length);
    callback_->Encoded(frame->image(), NULL, NULL);
    frames_.pop();
  }
}

int32_t WebrtcOpenH264VideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::Release() {
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::SetChannelParameters(uint32_t packetLoss,
						     int rtt) {
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoEncoder::SetRates(uint32_t newBitRate,
                                             uint32_t frameRate) {
  //update bitrate if needed
  int32_t existEncoderBitRate = 0;
  int rv = encoder_->GetOption(ENCODER_OPTION_BITRATE, &existEncoderBitRate);
  if (rv!=cmResultSuccess)
  {
    MOZ_MTLOG(ML_ERROR, "Error in Getting Existing Encoder Bandwidth: ReturnValue: " << rv);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  int32_t newEncoderBitRate = newBitRate*1000; //kbps->bps
  if ( existEncoderBitRate!=newEncoderBitRate ) {
    rv = encoder_->SetOption(ENCODER_OPTION_BITRATE, &newEncoderBitRate);
    MOZ_MTLOG(ML_INFO, "Update Encoder Bandwidth: ReturnValue: " << rv << " BitRate(kbps): " << newBitRate);
    if (rv!=cmResultSuccess)
    {
      MOZ_MTLOG(ML_ERROR, "Error in Setting Encoder Bandwidth: ReturnValue: " << rv);
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  //update framerate
  float newFrameRate = static_cast<float>(frameRate);
  rv = encoder_->SetOption(ENCODER_OPTION_FRAME_RATE, &newFrameRate);
  MOZ_MTLOG(ML_INFO, "Update Encoder Frame Rate: ReturnValue: " << rv << " FrameRate: " << frameRate);
  if (rv!=cmResultSuccess)
  {
    MOZ_MTLOG(ML_ERROR, "Error in Setting Encoder Frame Rate: ReturnValue: " << rv);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}



// Decoder.
WebrtcOpenH264VideoDecoder::WebrtcOpenH264VideoDecoder()
    : decoder_(nullptr),
      callback_(nullptr),
      mutex_("WebrtcOpenH264VideoDecoder") {
  nsIThread* thread;

  nsresult rv = NS_NewNamedThread("encoder-thread", &thread);
  MOZ_ASSERT(NS_SUCCEEDED(rv));

  thread_ = thread;
}

WebrtcOpenH264VideoDecoder::~WebrtcOpenH264VideoDecoder() {
  if (decoder_) {
    DestroyDecoder(decoder_);
  }
}

int32_t WebrtcOpenH264VideoDecoder::InitDecode(
    const webrtc::VideoCodec* codecSettings,
    int32_t numberOfCores) {
  long rv = CreateDecoder (&decoder_);
  if (rv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  SDecodingParam param;
  memset(&param, 0, sizeof(param));
  param.iOutputColorFormat = videoFormatI420;
  param.uiTargetDqLayer = UCHAR_MAX;  // TODO(ekr@rtfm.com): correct?
  param.uiEcActiveFlag = 1; // Error concealment on.
  param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;

  long lrv = decoder_->Initialize(&param);
  if (lrv) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Decode(
    const webrtc::EncodedImage& inputImage,
    bool missingFrames,
    const webrtc::RTPFragmentationHeader* fragmentation,
    const webrtc::CodecSpecificInfo*
    codecSpecificInfo,
    int64_t renderTimeMs) {
  SBufferInfo decoded;
  memset(&decoded, 0, sizeof(decoded));
  void *data[3] = {nullptr, nullptr, nullptr};
  MOZ_MTLOG(ML_DEBUG, "Decoding frame input length=" << inputImage._length);
  int rv = decoder_->DecodeFrame2(inputImage._buffer,
                                 inputImage._length,
                                 data,
                                 &decoded);
  if (rv) {
    MOZ_MTLOG(ML_ERROR, "Decoding error rv=" << rv);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }


  MutexAutoLock lock(mutex_);
  int width;
  int height;
  int ystride;
  int uvstride;

  width = decoded.UsrData.sSystemBuffer.iWidth;
  height = decoded.UsrData.sSystemBuffer.iHeight;
  ystride = decoded.UsrData.sSystemBuffer.iStride[0];
  uvstride = decoded.UsrData.sSystemBuffer.iStride[1];

  int len = width * height;

  if (len) {
    if (decoded_image_.CreateFrame(ystride * height, static_cast<uint8_t *>(data[0]),
                                   uvstride * height/2, static_cast<uint8_t *>(data[1]),
                                   uvstride * height/2, static_cast<uint8_t *>(data[2]),
                                   width, height,
                                   ystride, uvstride, uvstride
                                   ))
      return WEBRTC_VIDEO_CODEC_ERROR;
    decoded_image_.set_timestamp(inputImage._timeStamp);

    RUN_ON_THREAD(thread_,
                  // Shared pointer keeps the object live.
                  WrapRunnable(nsRefPtr<WebrtcOpenH264VideoDecoder>(this),
                               &WebrtcOpenH264VideoDecoder::RunCallback),
                  NS_DISPATCH_NORMAL);
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

void WebrtcOpenH264VideoDecoder::RunCallback() {
  MutexAutoLock lock(mutex_);

  callback_->Decoded(decoded_image_);
}

int32_t WebrtcOpenH264VideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}


int32_t WebrtcOpenH264VideoDecoder::Release() {
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t WebrtcOpenH264VideoDecoder::Reset() {
  return WEBRTC_VIDEO_CODEC_OK;
}

}
