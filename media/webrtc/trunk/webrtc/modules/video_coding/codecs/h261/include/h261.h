#ifndef WEBRTC_MODULES_VIDEO_CODING_CODECS_H261_INCLUDE_H261_H_
#define WEBRTC_MODULES_VIDEO_CODING_CODECS_H261_INCLUDE_H261_H_

#include "webrtc/modules/video_coding/codecs/interface/video_codec_interface.h"

namespace webrtc {

class H261Encoder : public VideoEncoder {
 public:
  static H261Encoder* Create();

  virtual ~H261Encoder() {};
};  // end of H261Encoder class


class H261Decoder : public VideoDecoder {
 public:
  static H261Decoder* Create();

  virtual ~H261Decoder() {};
};  // end of H261Decoder class
}  // namespace webrtc

#endif // WEBRTC_MODULES_VIDEO_CODING_CODECS_H261_INCLUDE_H261_H_
