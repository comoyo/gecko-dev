/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

namespace mozilla {
class YuvStamper {
 public:
  YuvStamper(uint32_t width, uint32_t height, uint32_t stride)
      : width_(width), height_(height), stride_(stride) {}

  bool Write(uint8_t *data, uint32_t value, uint32_t x=0, uint32_t y=0);


  template<typename T> static bool Write(uint32_t width,
                                         uint32_t height,
                                         uint32_t stride,
                                         uint8_t *data,
                                         const T& value,
                                         uint32_t x=0,
                                         uint32_t y=0) {
    YuvStamper stamper(width, height, stride);

    return stamper.Write(data, value, x, y);
  }

 private:
  bool WriteDigit(uint8_t* data, uint32_t x, uint32_t y, uint8_t digit);
  bool WritePixel(uint8_t* data, uint32_t x, uint32_t y);

  static const uint32_t kPixelSize = 3;
  static const uint32_t kDigitWidth = 6;
  static const uint32_t kDigitHeight = 7;

  uint32_t width_;
  uint32_t height_;
  uint32_t stride_;
};

}


