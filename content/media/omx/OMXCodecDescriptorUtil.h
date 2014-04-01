/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-*/
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef OMXCodecDescriptorUtil_h_
#define OMXCodecDescriptorUtil_h_

#include <stagefright/foundation/ABuffer.h>
#include <stagefright/MediaErrors.h>

#include <nsTArray.h>

namespace android {

// Generate decoder config blob using data provided by encoder. It supports 2
// formats: if aNAL is true, the generated blob will be in a NAL unit format
// needed by WebRTC for constructing RTP packets; otherwise it will be in
// decoder config descriptor format (defined in ISO/IEC 14496-15 5.2.4.1.1) for
// AVC/H.264 data in MP4 file.
status_t GenerateAVCDescriptorBlob(ABuffer* aData,
                                   nsTArray<uint8_t>* aOutputBuf,
                                   bool aNAL);

}

#endif // OMXCodecDescriptorUtil_h_