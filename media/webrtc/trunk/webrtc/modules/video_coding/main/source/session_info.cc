/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/modules/video_coding/main/source/session_info.h"
#include "webrtc/modules/video_coding/main/source/packet.h"
#include "webrtc/modules/rtp_rtcp/source/rtp_format_h264.h"

namespace webrtc {

// Used in determining whether a frame is decodable.
enum {kRttThreshold = 100};  // Not decodable if Rtt is lower than this.

// Do not decode frames if the number of packets is between these two
// thresholds.
static const float kLowPacketPercentageThreshold = 0.2f;
static const float kHighPacketPercentageThreshold = 0.8f;

VCMSessionInfo::VCMSessionInfo()
    : session_nack_(false),
      complete_(false),
      decodable_(false),
      frame_type_(kVideoFrameDelta),
      packets_(),
      empty_seq_num_low_(-1),
      empty_seq_num_high_(-1),
      first_packet_seq_num_(-1),
      last_packet_seq_num_(-1) {
}

void VCMSessionInfo::UpdateDataPointers(const uint8_t* old_base_ptr,
                                        const uint8_t* new_base_ptr) {
  for (PacketIterator it = packets_.begin(); it != packets_.end(); ++it)
    if ((*it).dataPtr != NULL) {
      assert(old_base_ptr != NULL && new_base_ptr != NULL);
      (*it).dataPtr = new_base_ptr + ((*it).dataPtr - old_base_ptr);
    }
}

int VCMSessionInfo::LowSequenceNumber() const {
  if (packets_.empty())
    return empty_seq_num_low_;
  return packets_.front().seqNum;
}

int VCMSessionInfo::HighSequenceNumber() const {
  if (packets_.empty())
    return empty_seq_num_high_;
  if (empty_seq_num_high_ == -1)
    return packets_.back().seqNum;
  return LatestSequenceNumber(packets_.back().seqNum, empty_seq_num_high_);
}

int VCMSessionInfo::PictureId() const {
  if (packets_.empty() ||
      packets_.front().codecSpecificHeader.codec != kRtpVideoVp8)
    return kNoPictureId;
  return packets_.front().codecSpecificHeader.codecHeader.VP8.pictureId;
}

int VCMSessionInfo::TemporalId() const {
  if (packets_.empty() ||
      packets_.front().codecSpecificHeader.codec != kRtpVideoVp8)
    return kNoTemporalIdx;
  return packets_.front().codecSpecificHeader.codecHeader.VP8.temporalIdx;
}

bool VCMSessionInfo::LayerSync() const {
  if (packets_.empty() ||
        packets_.front().codecSpecificHeader.codec != kRtpVideoVp8)
    return false;
  return packets_.front().codecSpecificHeader.codecHeader.VP8.layerSync;
}

int VCMSessionInfo::Tl0PicId() const {
  if (packets_.empty() ||
      packets_.front().codecSpecificHeader.codec != kRtpVideoVp8)
    return kNoTl0PicIdx;
  return packets_.front().codecSpecificHeader.codecHeader.VP8.tl0PicIdx;
}

bool VCMSessionInfo::NonReference() const {
  if (packets_.empty() ||
      packets_.front().codecSpecificHeader.codec != kRtpVideoVp8)
    return false;
  return packets_.front().codecSpecificHeader.codecHeader.VP8.nonReference;
}

void VCMSessionInfo::Reset() {
  session_nack_ = false;
  complete_ = false;
  decodable_ = false;
  frame_type_ = kVideoFrameDelta;
  packets_.clear();
  empty_seq_num_low_ = -1;
  empty_seq_num_high_ = -1;
  first_packet_seq_num_ = -1;
  last_packet_seq_num_ = -1;
}

int VCMSessionInfo::SessionLength() const {
  int length = 0;
  for (PacketIteratorConst it = packets_.begin(); it != packets_.end(); ++it)
    length += (*it).sizeBytes;
  return length;
}

int VCMSessionInfo::NumPackets() const {
  return packets_.size();
}

void VCMSessionInfo::CopyPacket(uint8_t* dst, const uint8_t* src, size_t len) {
  memcpy(dst, src, len);
}

void VCMSessionInfo::CopyWithStartCode(uint8_t* dst, const uint8_t* src, size_t len) {
  // H.264 Start Code is 2 or more bytes of 0 followed by 1 byte of 1.
  memset(dst, 0, RtpFormatH264::kStartCodeSize-1);
  dst[RtpFormatH264::kStartCodeSize-1] = 1;
  CopyPacket(dst + RtpFormatH264::kStartCodeSize, src, len);
}

int VCMSessionInfo::InsertBuffer(uint8_t* frame_buffer,
                                 PacketIterator packet_it) {
  VCMPacket& packet = *packet_it;

  // Advance to the offset into the frame buffer for this packet.
  for (PacketIterator it = packets_.begin(); it != packet_it; ++it)
    frame_buffer += (*it).sizeBytes;

  if (packet.codec == kVideoCodecH264) {
    // Calculate extra packet size needed for adding start codes,
    // and removing fragmentation and aggregation unit headers.
    size_t nalu_size;
    size_t all_nalu_size = 0;
    const uint8_t* nalu_ptr = packet.dataPtr;
    uint8_t nal_header = *nalu_ptr;
    uint8_t fu_header;
    switch (nal_header & RtpFormatH264::kTypeMask) {
      case RtpFormatH264::kFuA:
        fu_header = nalu_ptr[RtpFormatH264::kFuAHeaderOffset];
        if (fu_header & RtpFormatH264::kFragStartBit) {
          nal_header &= ~RtpFormatH264::kTypeMask; // Keep F/NRI bits
          nal_header |= fu_header & RtpFormatH264::kTypeMask; // Keep NAL type
          packet.sizeBytes -= RtpFormatH264::kFuAHeaderOffset;
          packet.dataPtr += RtpFormatH264::kFuAHeaderOffset;
          ShiftSubsequentPackets(packet_it, packet.sizeBytes +
                                 RtpFormatH264::kStartCodeSize);
          CopyWithStartCode(frame_buffer, packet.dataPtr, packet.sizeBytes);
          frame_buffer[RtpFormatH264::kStartCodeSize] = nal_header;
          packet.sizeBytes += RtpFormatH264::kStartCodeSize;
          packet.dataPtr = frame_buffer;
          packet.completeNALU = kNaluStart;
        } else {
          packet.sizeBytes -= RtpFormatH264::kFuAHeaderOffset +
                              RtpFormatH264::kFuAHeaderSize;
          packet.dataPtr += RtpFormatH264::kFuAHeaderOffset +
                            RtpFormatH264::kFuAHeaderSize;
          ShiftSubsequentPackets(packet_it, packet.sizeBytes);
          CopyPacket(frame_buffer, packet.dataPtr, packet.sizeBytes);
          packet.dataPtr = frame_buffer;
          if (fu_header & RtpFormatH264::kFragEndBit) {
            packet.completeNALU = kNaluEnd;
          } else {
            packet.completeNALU = kNaluIncomplete;
          }
        }
        break;
      case RtpFormatH264::kStapA:
        packet.sizeBytes -= RtpFormatH264::kStapAHeaderOffset;
        packet.dataPtr += RtpFormatH264::kStapAHeaderOffset;
        for (nalu_ptr = packet.dataPtr;
             nalu_ptr < packet.dataPtr + packet.sizeBytes;
             nalu_ptr += nalu_size + RtpFormatH264::kAggUnitLengthSize) {
          nalu_size = (nalu_ptr[0] << 8) + nalu_ptr[1];
          all_nalu_size += nalu_size + RtpFormatH264::kStartCodeSize;
        }
        if (nalu_ptr > packet.dataPtr + packet.sizeBytes) {
          // malformed packet
          packet.completeNALU = kNaluIncomplete;
          return -1;
        }
        ShiftSubsequentPackets(packet_it, all_nalu_size);
        for (nalu_ptr = packet.dataPtr;
             nalu_ptr < packet.dataPtr + packet.sizeBytes;
             nalu_ptr += nalu_size + RtpFormatH264::kAggUnitLengthSize) {
          nalu_size = (nalu_ptr[0] << 8) + nalu_ptr[1];
          CopyWithStartCode(frame_buffer, nalu_ptr+2, nalu_size);
          frame_buffer += nalu_size + RtpFormatH264::kStartCodeSize;
        }
        packet.sizeBytes = all_nalu_size;
        packet.dataPtr = frame_buffer - all_nalu_size;
        packet.completeNALU = kNaluComplete;
        break;
      default:
        ShiftSubsequentPackets(packet_it, packet.sizeBytes +
                               RtpFormatH264::kStartCodeSize);
        CopyWithStartCode(frame_buffer, packet.dataPtr, packet.sizeBytes);
        packet.sizeBytes += RtpFormatH264::kStartCodeSize;
        packet.dataPtr = frame_buffer;
        packet.completeNALU = kNaluComplete;
        break;
    } // switch nal_type
  } else { // not H.264
    ShiftSubsequentPackets(packet_it, packet.sizeBytes);
    CopyPacket(frame_buffer, packet.dataPtr, packet.sizeBytes);
    packet.dataPtr = frame_buffer;
  }
  return packet.sizeBytes;
}

void VCMSessionInfo::ShiftSubsequentPackets(PacketIterator it,
                                            int steps_to_shift) {
  ++it;
  if (it == packets_.end())
    return;
  uint8_t* first_packet_ptr = const_cast<uint8_t*>((*it).dataPtr);
  int shift_length = 0;
  // Calculate the total move length and move the data pointers in advance.
  for (; it != packets_.end(); ++it) {
    shift_length += (*it).sizeBytes;
    if ((*it).dataPtr != NULL)
      (*it).dataPtr += steps_to_shift;
  }
  memmove(first_packet_ptr + steps_to_shift, first_packet_ptr, shift_length);
}

void VCMSessionInfo::UpdateCompleteSession() {
  if (HaveFirstPacket() && HaveLastPacket()) {
    // Do we have all the packets in this session?
    bool complete_session = true;
    PacketIterator it = packets_.begin();
    PacketIterator prev_it = it;
    ++it;
    for (; it != packets_.end(); ++it) {
      if (!InSequence(it, prev_it)) {
        complete_session = false;
        break;
      }
      prev_it = it;
    }
    complete_ = complete_session;
  }
}

void VCMSessionInfo::UpdateDecodableSession(const FrameData& frame_data) {
  // Irrelevant if session is already complete or decodable
  if (complete_ || decodable_)
    return;
  // TODO(agalusza): Account for bursty loss.
  // TODO(agalusza): Refine these values to better approximate optimal ones.
  if (frame_data.rtt_ms < kRttThreshold
      || frame_type_ == kVideoFrameKey
      || !HaveFirstPacket()
      || (NumPackets() <= kHighPacketPercentageThreshold
                          * frame_data.rolling_average_packets_per_frame
          && NumPackets() > kLowPacketPercentageThreshold
                            * frame_data.rolling_average_packets_per_frame))
    return;

  decodable_ = true;
}

bool VCMSessionInfo::complete() const {
  return complete_;
}

bool VCMSessionInfo::decodable() const {
  return decodable_;
}

// Find the end of the NAL unit which the packet pointed to by |packet_it|
// belongs to. Returns an iterator to the last packet of the frame if the end
// of the NAL unit wasn't found.
VCMSessionInfo::PacketIterator VCMSessionInfo::FindNaluEnd(
    PacketIterator packet_it) const {
  if ((*packet_it).completeNALU == kNaluEnd ||
      (*packet_it).completeNALU == kNaluComplete) {
    return packet_it;
  }
  // Find the end of the NAL unit.
  for (; packet_it != packets_.end(); ++packet_it) {
    if (((*packet_it).completeNALU == kNaluComplete &&
        (*packet_it).sizeBytes > 0) ||
        // Found next NALU.
        (*packet_it).completeNALU == kNaluStart)
      return --packet_it;
    if ((*packet_it).completeNALU == kNaluEnd)
      return packet_it;
  }
  // The end wasn't found.
  return --packet_it;
}

int VCMSessionInfo::DeletePacketData(PacketIterator start,
                                     PacketIterator end) {
  int bytes_to_delete = 0;  // The number of bytes to delete.
  PacketIterator packet_after_end = end;
  ++packet_after_end;

  // Get the number of bytes to delete.
  // Clear the size of these packets.
  for (PacketIterator it = start; it != packet_after_end; ++it) {
    bytes_to_delete += (*it).sizeBytes;
    (*it).sizeBytes = 0;
    (*it).dataPtr = NULL;
  }
  if (bytes_to_delete > 0)
    ShiftSubsequentPackets(end, -bytes_to_delete);
  return bytes_to_delete;
}

int VCMSessionInfo::BuildVP8FragmentationHeader(
    uint8_t* frame_buffer,
    int frame_buffer_length,
    RTPFragmentationHeader* fragmentation) {
  int new_length = 0;
  // Allocate space for max number of partitions
  fragmentation->VerifyAndAllocateFragmentationHeader(kMaxVP8Partitions);
  fragmentation->fragmentationVectorSize = 0;
  memset(fragmentation->fragmentationLength, 0,
         kMaxVP8Partitions * sizeof(uint32_t));
  if (packets_.empty())
      return new_length;
  PacketIterator it = FindNextPartitionBeginning(packets_.begin());
  while (it != packets_.end()) {
    const int partition_id =
        (*it).codecSpecificHeader.codecHeader.VP8.partitionId;
    PacketIterator partition_end = FindPartitionEnd(it);
    fragmentation->fragmentationOffset[partition_id] =
        (*it).dataPtr - frame_buffer;
    assert(fragmentation->fragmentationOffset[partition_id] <
           static_cast<uint32_t>(frame_buffer_length));
    fragmentation->fragmentationLength[partition_id] =
        (*partition_end).dataPtr + (*partition_end).sizeBytes - (*it).dataPtr;
    assert(fragmentation->fragmentationLength[partition_id] <=
           static_cast<uint32_t>(frame_buffer_length));
    new_length += fragmentation->fragmentationLength[partition_id];
    ++partition_end;
    it = FindNextPartitionBeginning(partition_end);
    if (partition_id + 1 > fragmentation->fragmentationVectorSize)
      fragmentation->fragmentationVectorSize = partition_id + 1;
  }
  // Set all empty fragments to start where the previous fragment ends,
  // and have zero length.
  if (fragmentation->fragmentationLength[0] == 0)
      fragmentation->fragmentationOffset[0] = 0;
  for (int i = 1; i < fragmentation->fragmentationVectorSize; ++i) {
    if (fragmentation->fragmentationLength[i] == 0)
      fragmentation->fragmentationOffset[i] =
          fragmentation->fragmentationOffset[i - 1] +
          fragmentation->fragmentationLength[i - 1];
    assert(i == 0 ||
           fragmentation->fragmentationOffset[i] >=
           fragmentation->fragmentationOffset[i - 1]);
  }
  assert(new_length <= frame_buffer_length);
  return new_length;
}

int VCMSessionInfo::BuildH261FragmentationHeader(
    uint8_t* frame_buffer,
    int frame_buffer_length,
    RTPFragmentationHeader* fragmentation) {
  int new_length = 0;
  // Allocate space for all packets
  int num_packets = packets_.size();
  fragmentation->VerifyAndAllocateFragmentationHeader(num_packets);
  fragmentation->fragmentationVectorSize = 0;
  memset(fragmentation->fragmentationLength, 0,
         num_packets * sizeof(uint32_t));
  if (packets_.empty())
      return new_length;
  int fragment_id = 0;
  PacketIterator it = packets_.begin();
  while (it != packets_.end()) {
    fragmentation->fragmentationOffset[fragment_id] =
        (*it).dataPtr - frame_buffer;
    assert(fragmentation->fragmentationOffset[fragment_id] <
           static_cast<uint32_t>(frame_buffer_length));
    fragmentation->fragmentationLength[fragment_id] =
        (*it).sizeBytes;
    assert(fragmentation->fragmentationLength[fragment_id] <=
           static_cast<uint32_t>(frame_buffer_length));
    new_length += fragmentation->fragmentationLength[fragment_id];
    ++it;
    if (++fragment_id > fragmentation->fragmentationVectorSize)
      fragmentation->fragmentationVectorSize = fragment_id;
  }
  // Set all empty fragments to start where the previous fragment ends,
  // and have zero length.
  if (fragmentation->fragmentationLength[0] == 0)
      fragmentation->fragmentationOffset[0] = 0;
  for (int i = 1; i < fragmentation->fragmentationVectorSize; ++i) {
    if (fragmentation->fragmentationLength[i] == 0)
      fragmentation->fragmentationOffset[i] =
          fragmentation->fragmentationOffset[i - 1] +
          fragmentation->fragmentationLength[i - 1];
    assert(i == 0 ||
           fragmentation->fragmentationOffset[i] >=
           fragmentation->fragmentationOffset[i - 1]);
  }
  assert(new_length <= frame_buffer_length);
  return new_length;
}

VCMSessionInfo::PacketIterator VCMSessionInfo::FindNextPartitionBeginning(
    PacketIterator it) const {
  while (it != packets_.end()) {
    if ((*it).codecSpecificHeader.codecHeader.VP8.beginningOfPartition) {
      return it;
    }
    ++it;
  }
  return it;
}

VCMSessionInfo::PacketIterator VCMSessionInfo::FindPartitionEnd(
    PacketIterator it) const {
  assert((*it).codec == kVideoCodecVP8);
  PacketIterator prev_it = it;
  const int partition_id =
      (*it).codecSpecificHeader.codecHeader.VP8.partitionId;
  while (it != packets_.end()) {
    bool beginning =
        (*it).codecSpecificHeader.codecHeader.VP8.beginningOfPartition;
    int current_partition_id =
        (*it).codecSpecificHeader.codecHeader.VP8.partitionId;
    bool packet_loss_found = (!beginning && !InSequence(it, prev_it));
    if (packet_loss_found ||
        (beginning && current_partition_id != partition_id)) {
      // Missing packet, the previous packet was the last in sequence.
      return prev_it;
    }
    prev_it = it;
    ++it;
  }
  return prev_it;
}

bool VCMSessionInfo::InSequence(const PacketIterator& packet_it,
                                const PacketIterator& prev_packet_it) {
  // If the two iterators are pointing to the same packet they are considered
  // to be in sequence.
  return (packet_it == prev_packet_it ||
      (static_cast<uint16_t>((*prev_packet_it).seqNum + 1) ==
          (*packet_it).seqNum));
}

int VCMSessionInfo::MakeDecodable() {
  int return_length = 0;
  if (packets_.empty()) {
    return 0;
  }
  PacketIterator it = packets_.begin();
  // Make sure we remove the first NAL unit if it's not decodable.
  if ((*it).completeNALU == kNaluIncomplete ||
      (*it).completeNALU == kNaluEnd) {
    PacketIterator nalu_end = FindNaluEnd(it);
    return_length += DeletePacketData(it, nalu_end);
    it = nalu_end;
  }
  PacketIterator prev_it = it;
  // Take care of the rest of the NAL units.
  for (; it != packets_.end(); ++it) {
    bool start_of_nalu = ((*it).completeNALU == kNaluStart ||
        (*it).completeNALU == kNaluComplete);
    if (!start_of_nalu && !InSequence(it, prev_it)) {
      // Found a sequence number gap due to packet loss.
      PacketIterator nalu_end = FindNaluEnd(it);
      return_length += DeletePacketData(it, nalu_end);
      it = nalu_end;
    }
    prev_it = it;
  }
  return return_length;
}

void VCMSessionInfo::SetNotDecodableIfIncomplete() {
  // We don't need to check for completeness first because the two are
  // orthogonal. If complete_ is true, decodable_ is irrelevant.
  decodable_ = false;
}

bool
VCMSessionInfo::HaveFirstPacket() const {
  return !packets_.empty() && (first_packet_seq_num_ != -1);
}

bool
VCMSessionInfo::HaveLastPacket() const {
  return !packets_.empty() && (last_packet_seq_num_ != -1);
}

bool
VCMSessionInfo::session_nack() const {
  return session_nack_;
}

int VCMSessionInfo::InsertPacket(const VCMPacket& packet,
                                 uint8_t* frame_buffer,
                                 VCMDecodeErrorMode decode_error_mode,
                                 const FrameData& frame_data) {
  if (packet.frameType == kFrameEmpty) {
    // Update sequence number of an empty packet.
    // Only media packets are inserted into the packet list.
    InformOfEmptyPacket(packet.seqNum);
    return 0;
  }

  if (packets_.size() == kMaxPacketsInSession) {
    return -1;
  }

  // Find the position of this packet in the packet list in sequence number
  // order and insert it. Loop over the list in reverse order.
  ReversePacketIterator rit = packets_.rbegin();
  for (; rit != packets_.rend(); ++rit)
    if (LatestSequenceNumber(packet.seqNum, (*rit).seqNum) == packet.seqNum) {
      break;
    }

  // Check for duplicate packets.
  if (rit != packets_.rend() &&
      (*rit).seqNum == packet.seqNum && (*rit).sizeBytes > 0) {
    return -2;
  }

  PacketIterator packet_list_it;

  if (packet.codec == kVideoCodecH264) {
    // H.264 can have leading or trailing non-VCL (Video Coding Layer)
    // NALUs, such as SPS/PPS/SEI and others.  Also, the RTP marker bit is
    // not reliable for the last packet of a frame (RFC 6184 5.1 - "Decoders
    // [] MUST NOT rely on this property"), so allow out-of-order packets to
    // update the first and last seq# range.  Also mark as a key frame if
    // any packet is of that type.
    if (frame_type_ != kVideoFrameKey) {
      frame_type_ = packet.frameType;
    }
    if ((!HaveFirstPacket() ||
        IsNewerSequenceNumber(first_packet_seq_num_, packet.seqNum)) &&
        packet.isFirstPacket) {
      first_packet_seq_num_ = packet.seqNum;
    }
    // Note: the code does *not* currently handle the Marker bit being totally
    // absent from a frame.  It does not, however, depend on it being on the last
    // packet of the 'frame'/'session'.
    if ((!HaveLastPacket() && packet.markerBit) ||
        (HaveLastPacket() &&
        IsNewerSequenceNumber(packet.seqNum, last_packet_seq_num_))) {
      last_packet_seq_num_ = packet.seqNum;
    }
  } else {
    // Only insert media packets between first and last packets (when available).
    // Placing check here, as to properly account for duplicate packets.
    // Check if this is first packet (only valid for some codecs)
    // Should only be set for one packet per session.
    if (packet.isFirstPacket && first_packet_seq_num_ == -1) {
      // The first packet in a frame signals the frame type.
      frame_type_ = packet.frameType;
      // Store the sequence number for the first packet.
      first_packet_seq_num_ = static_cast<int>(packet.seqNum);
    } else if (first_packet_seq_num_ != -1 &&
               !IsNewerSequenceNumber(packet.seqNum, first_packet_seq_num_)) {
      //LOG(LS_WARNING) << "Received packet with a sequence number which is out "
      //                 "of frame boundaries";
      return -3;
    } else if (frame_type_ == kFrameEmpty && packet.frameType != kFrameEmpty) {
      // Update the frame type with the type of the first media packet.
      // TODO(mikhal): Can this trigger?
      frame_type_ = packet.frameType;
    }

    // Track the marker bit, should only be set for one packet per session.
    if (packet.markerBit && last_packet_seq_num_ == -1) {
      last_packet_seq_num_ = static_cast<int>(packet.seqNum);
    } else if (last_packet_seq_num_ != -1 &&
               IsNewerSequenceNumber(packet.seqNum, last_packet_seq_num_)) {
      //LOG(LS_WARNING) << "Received packet with a sequence number which is out "
      //                 "of frame boundaries";
      return -3;
    }
  }

  // The insert operation invalidates the iterator |rit|.
  packet_list_it = packets_.insert(rit.base(), packet);

  int returnLength = InsertBuffer(frame_buffer, packet_list_it);
  UpdateCompleteSession();
  if (decode_error_mode == kWithErrors)
    decodable_ = true;
  else if (decode_error_mode == kSelectiveErrors)
    UpdateDecodableSession(frame_data);
  return returnLength;
}

void VCMSessionInfo::InformOfEmptyPacket(uint16_t seq_num) {
  // Empty packets may be FEC or filler packets. They are sequential and
  // follow the data packets, therefore, we should only keep track of the high
  // and low sequence numbers and may assume that the packets in between are
  // empty packets belonging to the same frame (timestamp).
  if (empty_seq_num_high_ == -1)
    empty_seq_num_high_ = seq_num;
  else
    empty_seq_num_high_ = LatestSequenceNumber(seq_num, empty_seq_num_high_);
  if (empty_seq_num_low_ == -1 || IsNewerSequenceNumber(empty_seq_num_low_,
                                                        seq_num))
    empty_seq_num_low_ = seq_num;
}

}  // namespace webrtc
