
#include "webrtc/modules/video_coding/codecs/h261/h261_impl.h"

#include <iostream>
#include <cmath>

#include "webrtc/common_video/libyuv/include/webrtc_libyuv.h"
#include "webrtc/system_wrappers/interface/trace_event.h"

#define CIF_WIDTH   352
#define CIF_HEIGHT  288
#define QCIF_WIDTH  176
#define QCIF_HEIGHT 144

#define BEST_ENCODER_QUALITY     1
#define WORST_ENCODER_QUALITY   31
#define DEFAULT_ENCODER_QUALITY  5

#define DEFAULT_FILL_LEVEL       5

namespace webrtc {

H261Encoder* H261Encoder::Create() {
  return new H261EncoderImpl();
}

H261EncoderImpl::H261EncoderImpl()
    : encoded_complete_callback_(NULL),
      encoder_(NULL),
      inited_(false),
      temp_buffer_(NULL),
      encoding_size_changed_(false) {
  memset(&codec_, 0, sizeof(codec_));
}

H261EncoderImpl::~H261EncoderImpl() {
  Release();
}

int H261EncoderImpl::Release() {

  if (encoder_ != NULL) {
    delete encoder_;
    encoder_ = NULL;
  }
  if (encoded_image_._buffer != NULL) {
    delete [] encoded_image_._buffer;
    encoded_image_._buffer = NULL;
  }
  if (temp_buffer_ != NULL) {
    delete [] temp_buffer_;
    temp_buffer_ = NULL;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int H261EncoderImpl::InitEncode(const VideoCodec* inst,
               int number_of_cores,
               uint32_t max_payload_size) {
  if (inst == NULL) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->maxFramerate < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  // allow zero to represent an unspecified maxBitRate
  if (inst->maxBitrate > 0 && inst->startBitrate > inst->maxBitrate) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->width < 1 || inst->height < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (number_of_cores < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  int retVal = Release();
  if (retVal != WEBRTC_VIDEO_CODEC_OK) {
    return retVal;
  }
  if (&codec_ != inst) {
    codec_ = *inst;
  }

  encoder_ = new P64Encoder(DEFAULT_ENCODER_QUALITY, DEFAULT_FILL_LEVEL);
  if ((retVal = SetRates(codec_.startBitrate, 30)) != WEBRTC_VIDEO_CODEC_OK) {
    return retVal;
  }
  force_iframe_ = false;
  capture_width_ = capture_height_ = 0;
  video_quality_ = DEFAULT_ENCODER_QUALITY;

  inited_ = true;

  return codec_.startBitrate;
}

const uint8_t* H261EncoderImpl::PackI420Frame(const I420VideoFrame& frame) {
  const uint8_t* cur_src = frame.buffer(kYPlane);
  uint8_t* cur_dest = temp_buffer_;
  assert(frame.stride(kYPlane) == frame.width() && "Dude, the y plane is not packed!");
  int cur_size = frame.width() * frame.height();
  memcpy(cur_dest, cur_src, cur_size);

  cur_dest += cur_size;
  cur_src = frame.buffer(kUPlane);
  assert(frame.stride(kUPlane) == frame.width() / 2 && "Dude, the u plane is not packed!");
  cur_size = frame.width() / 2 * frame.height() / 2;
  memcpy(cur_dest, cur_src, cur_size);

  cur_dest += cur_size;
  cur_src = frame.buffer(kVPlane);
  assert(frame.stride(kVPlane) == frame.width() / 2 && "Dude, the v plane is not packed!");
  // v plane has same size as u plane
  memcpy(cur_dest, cur_src, cur_size);

  return temp_buffer_;
}

int H261EncoderImpl::Encode(const I420VideoFrame& input_image,
           const CodecSpecificInfo* codec_specific_info,
           const std::vector<VideoFrameType>* frame_types) {
  TRACE_EVENT1("webrtc", "H261::Encode", "timestamp", input_image.timestamp());

  if (!inited_) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (input_image.IsZeroSize()) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (encoded_complete_callback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  VideoFrameType frame_type = kDeltaFrame;
  // We only support one stream at the moment.
  if (frame_types && frame_types->size() > 0) {
    frame_type = (*frame_types)[0];
  }

  if (input_image.width() != capture_width_ ||
      input_image.height() != capture_height_ ||
      encoding_size_changed_) {

    capture_width_ = input_image.width();
    capture_height_ = input_image.height();

    if (temp_buffer_ != NULL) {
      delete [] temp_buffer_;
    }
    temp_buffer_ = new uint8_t[capture_width_ * capture_height_ * 3 / 2];

    double crop_aspect_ratio = (double) CIF_WIDTH / CIF_HEIGHT;
    crop_width_ = capture_height_ * crop_aspect_ratio;
    crop_height_ = capture_height_;

    if (crop_width_ > capture_width_) {
      // In case input_image is narrower than CIF we crop the height instead
      crop_width_ = capture_width_;
      crop_height_ = capture_width_ * crop_aspect_ratio;
    }

    cropped_image_.CreateEmptyFrame(crop_width_, crop_height_,
                                    crop_width_,
                                    (crop_width_ + 1) / 2,
                                    (crop_width_ + 1) / 2);


    switch (scaler_.Set(crop_width_, crop_height_,
                       frame_width_, frame_height_,
                       kI420, kI420,
                       kScaleBilinear)) {
      case -1:
        return WEBRTC_VIDEO_CODEC_ERROR;
      case -2:
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
  }

  // XXX(pehrsons) Copying occurs here
  const uint8_t* buffer = PackI420Frame(input_image);
  ConvertToI420(kI420, buffer,
                (capture_width_ - crop_width_) / 2,
                (capture_height_ - crop_height_) / 2,
                capture_width_, capture_height_,
                capture_width_ * capture_height_ * 3 / 2,
                kRotateNone, &cropped_image_);

  if (scaler_.Scale(cropped_image_, &scaled_image_) != 0) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  {
    uint8_t* cur_dest = encoder_->GetFramePtr();
    assert(cur_dest && "H261 FramePtr null check");

  // XXX(pehrsons) Copying occurs here
    const uint8_t* packed_buffer = PackI420Frame(scaled_image_);
    memcpy(cur_dest, packed_buffer, frame_width_ * frame_height_ * 3 / 2);
  }

  switch (frame_type) {
    case kKeyFrame:
      break;
    case kDeltaFrame:
      break;
    case kGoldenFrame:
      frame_type = kKeyFrame;
      break;
    case kAltRefFrame:
      frame_type = kDeltaFrame;
      break;
    case kSkipFrame:
      return WEBRTC_VIDEO_CODEC_OK;
    default:
      return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  if (frame_type == kKeyFrame || encoding_size_changed_) {
    encoder_->FastUpdatePicture();
  }


  encoder_->PreProcessOneFrame();

  EncodeAndSendNextFrame(input_image.timestamp(),
                         5, //input_image.render_time_ms());
                         frame_type);

  encoding_size_changed_ = false;

  return WEBRTC_VIDEO_CODEC_OK;
}

int H261EncoderImpl::RegisterEncodeCompleteCallback(EncodedImageCallback* callback) {
  encoded_complete_callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int H261EncoderImpl::SetChannelParameters(uint32_t packet_loss, int rtt) {
  return WEBRTC_VIDEO_CODEC_OK;
}

static int GetQCIFLevel(uint32_t new_bits_per_frame) {
  // Based on the following data:
  // http://www.wolframalpha.com/input/?i=quadratic+fit+%7B%7B1300%2F30%2C+1%7D%2C+%7B1000%2F30%2C+5%7D%2C+%7B640%2F30%2C+10%7D%2C+%7B500%2F30%2C+15%7D%2C+%7B400%2F30%2C+20%7D%2C+%7B320%2F30%2C+25%7D%2C+%7B260%2F30%2C+31%7D%7D

  if (new_bits_per_frame > 67) {
    return 1;
  }

  double x = new_bits_per_frame;
  double level = 0.0304503 * std::pow(x, 2) - 2.3572 * x + 47.1143;
  int result = round(level);
  result = std::min(result, 31);
  result = std::max(result, 1);
  return result;
}

static int GetCIFLevel(uint32_t new_bits_per_frame) {
  // Based on the following data:
  // http://www.wolframalpha.com/input/?i=quadratic+fit+%7B%7B2000%2F30%2C+1%7D%2C+%7B1600%2F30%2C+5%7D%2C+%7B1300%2F30%2C+10%7D%2C+%7B800%2F30%2C+15%7D%2C+%7B640%2F30%2C+20%7D%2C+%7B480%2F30%2C+25%7D%2C+%7B400%2F30%2C+31%7D%7D

  if (new_bits_per_frame > 67) {
    return 1;
  }

  double x = new_bits_per_frame;
  double level = 0.00802894 * std::pow(x, 2) - 1.13647 * x + 42.0093;
  int result = round(level);
  result = std::min(result, 31);
  result = std::max(result, 1);
  return result;
}

int H261EncoderImpl::SetSize(int width, int height) {

  if (frame_width_ == width && frame_height_ == height) {
    return WEBRTC_VIDEO_CODEC_OK;
  }

  // Reserve 100 extra bytes for overhead at small resolutions.
  uint32_t size = CalcBufferSize(kI420, width, height) + 100;

  // allocate memory for encoded image
  if (encoded_image_._buffer != NULL) {
    delete [] encoded_image_._buffer;
  }

  encoded_image_._size = size;
  encoded_image_._buffer = new uint8_t[size];

  encoded_image_._encodedWidth = width;
  encoded_image_._encodedHeight = height;

  frame_width_ = width;
  frame_height_ = height;
  encoder_->SetSize(frame_width_, frame_height_);

  encoding_size_changed_ = true;

  return WEBRTC_VIDEO_CODEC_OK;
}

int H261EncoderImpl::SetQCIFSize() {
  return SetSize(QCIF_WIDTH, QCIF_HEIGHT);
}

int H261EncoderImpl::SetCIFSize() {
  return SetSize(CIF_WIDTH, CIF_HEIGHT);
}

int H261EncoderImpl::SetRates(uint32_t new_bitrate_kbit, uint32_t frame_rate) {
  double new_bits_per_frame = new_bitrate_kbit / (double) frame_rate;

  int retVal;
  if (new_bits_per_frame < 20) {
    video_quality_ = GetQCIFLevel(new_bits_per_frame);
    retVal = SetQCIFSize();
  } else {
    video_quality_ = GetCIFLevel(new_bits_per_frame);
    retVal = SetCIFSize();
  }

  if (retVal != WEBRTC_VIDEO_CODEC_OK) {
    return retVal;
  }

  encoder_->SetQualityLevel(video_quality_);
  return WEBRTC_VIDEO_CODEC_OK;
}

H261Decoder* H261Decoder::Create() {
  return new H261DecoderImpl();
}

H261DecoderImpl::H261DecoderImpl()
    : decoded_complete_callback_(NULL),
      decoder_(NULL) {
  memset(&codec_, 0, sizeof(codec_));
}

H261DecoderImpl::~H261DecoderImpl() {
  Release();
}

int H261DecoderImpl::Release() {
  if (decoder_ != NULL) {
    delete decoder_;
    decoder_ = NULL;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int H261DecoderImpl::InitDecode(const VideoCodec* inst, int number_of_cores) {
  if (inst == NULL) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->maxFramerate < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  // allow zero to represent an unspecified maxBitRate
  if (inst->maxBitrate > 0 && inst->startBitrate > inst->maxBitrate) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (inst->width < 1 || inst->height < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  if (number_of_cores < 1) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  int retVal = Release();
  if (retVal < 0) {
    return retVal;
  }

  decoder_ = new FullP64Decoder();
  frame_width_ = frame_height_ = 0;

  return WEBRTC_VIDEO_CODEC_OK;
}

int H261DecoderImpl::Decode(const EncodedImage& input_image,
           bool missing_frames,
           const RTPFragmentationHeader* fragmentation,
           const CodecSpecificInfo* codec_specific_info,
           int64_t render_time_ms) {

  if (decoded_complete_callback_ == NULL) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  if (!decoder_->decode(input_image._buffer,
                        input_image._length,
                        missing_frames)) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  //Check for a resize - can change at any time!
  if (frame_width_ != (unsigned)decoder_->width() ||
      frame_height_ != (unsigned)decoder_->height()) {

    frame_width_ = decoder_->width();
    frame_height_ = decoder_->height();
  }

  decoder_->sync();

  {
    uint8_t* y_src = decoder_->GetFramePtr();
    assert(y_src && "H261 FramePtr null check");
    int y_stride = frame_width_;
    int y_size = y_stride * frame_height_;

    uint8_t* u_src = y_src + y_size;
    int u_stride = frame_width_ / 2;
    int u_size = u_stride * frame_height_ / 2;

    uint8_t* v_src = u_src + u_size;
    int v_stride = u_stride;
    int v_size = u_size;

    if (decoded_image_.CreateFrame(y_size, y_src,
                                   u_size, u_src,
                                   v_size, v_src,
                                   frame_width_, frame_height_,
                                   y_stride,
                                   u_stride,
                                   v_stride) != 0) {
      return WEBRTC_VIDEO_CODEC_ERROR;
    }

    decoded_image_.set_timestamp(input_image._timeStamp);
    decoded_image_.set_render_time_ms(render_time_ms);
  }

  if (decoded_complete_callback_->Decoded(decoded_image_) != 0) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  decoder_->resetndblk();

  return WEBRTC_VIDEO_CODEC_OK;
}

int H261DecoderImpl::RegisterDecodeCompleteCallback(DecodedImageCallback* callback) {
  decoded_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int H261DecoderImpl::Reset() {
  return WEBRTC_VIDEO_CODEC_OK;
}

VideoDecoder* H261DecoderImpl::Copy() {
  return NULL;
}

}
