/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <stdint.h>
#include <string.h>

#include "logging.h"
#include "nspr.h"
#include "YuvStamper.h"

namespace mozilla {

static unsigned char DIGIT_0 [] =
{
 0, 0, 1, 1, 0, 0,
 0, 1, 0, 0, 1, 0,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 0, 1, 0, 0, 1, 0,
 0, 0, 1, 1, 0, 0
};

static unsigned char DIGIT_1 [] =
{
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 0, 1, 0, 0,
};

static unsigned char DIGIT_2 [] =
{
 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 0,
 1, 0, 0, 0, 0, 0,
 1, 0, 0, 0, 0, 0,
 0, 1, 1, 1, 1, 1,
};

static unsigned char DIGIT_3 [] =
{
 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 1, 1, 1, 1, 1, 0,
};

static unsigned char DIGIT_4 [] =
{
 0, 1, 0, 0, 0, 1,
 0, 1, 0, 0, 0, 1,
 0, 1, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1
};

static unsigned char DIGIT_5 [] =
{
 0, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0,
 1, 0, 0, 0, 0, 0,
 0, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 1, 1, 1, 1, 1, 0,
};

static unsigned char DIGIT_6 [] =
{
 0, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0,
 1, 0, 0, 0, 0, 0,
 1, 1, 1, 1, 1, 0,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 0,
};

static unsigned char DIGIT_7 [] =
{
 1, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 1, 0,
 0, 0, 0, 1, 0, 0,
 0, 0, 1, 0, 0, 0,
 0, 1, 0, 0, 0, 0,
 1, 0, 0, 0, 0, 0
};

static unsigned char DIGIT_8 [] =
{
 0, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 0,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 0
};


static unsigned char DIGIT_9 [] =
{
 0, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 1,
 0, 0, 0, 0, 0, 1,
 0, 1, 1, 1, 1, 0
};

static unsigned char *DIGITS[] = {
    DIGIT_0,
    DIGIT_1,
    DIGIT_2,
    DIGIT_3,
    DIGIT_4,
    DIGIT_5,
    DIGIT_6,
    DIGIT_7,
    DIGIT_8,
    DIGIT_9
};

bool YuvStamper::WritePixel(uint8_t *data, uint32_t x, uint32_t y) {
  if (x > width_)
    return false;

  if (y > height_)
    return false;

  uint8_t *ptr = &data[y * stride_ + x];
  *ptr = (*ptr > 96) ? 0 : 0xff;

  return true;
}


bool YuvStamper::WriteDigit(uint8_t* data, uint32_t x, uint32_t y, uint8_t digit) {
  if (digit > 9)
    return false;

  unsigned char *dig = DIGITS[digit];

  for (uint32_t row = 0; row < kDigitHeight; ++row) {
    for (uint32_t col = 0; col < kDigitWidth; ++col) {
      if (dig[(row * kDigitWidth) + col]) {
        for (uint32_t xx=0; xx < kPixelSize; ++xx) {
          for (uint32_t yy=0; yy < kPixelSize; ++yy) {
            if (!WritePixel(data, x + (col * kPixelSize) + xx, y + (row * kPixelSize) + yy))
              return false;
          }
        }
      }
    }
  }

  return true;
}

bool YuvStamper::Write(uint8_t *data, uint32_t value, uint32_t x, uint32_t y) {
  char buf[20];

  PR_snprintf(buf, sizeof(buf), "%u", value);

  for (size_t i=0; i<strlen(buf); ++i) {
    if (!WriteDigit(data, x, y, buf[i] - '0'))
      return false;

    x += kPixelSize * (kDigitWidth + 1);
  }

  return true;
}

}










