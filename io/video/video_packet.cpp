#include "video_packet.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace io
{
namespace
{
void put(uint8_t * dst, uint64_t value, size_t size)
{
  for (size_t i = 0; i < size; ++i) dst[i] = static_cast<uint8_t>(value >> (8 * i));
}

uint64_t get(const uint8_t * src, size_t size)
{
  uint64_t value = 0;
  for (size_t i = 0; i < size; ++i) value |= uint64_t(src[i]) << (8 * i);
  return value;
}
}  // namespace

std::vector<VideoFragment> fragment_video(
  const std::vector<uint8_t> & encoded, VideoFrameInfo info, size_t block_max_bytes)
{
  if (block_max_bytes <= kVideoHeaderSize || block_max_bytes > kVideoDataSize) {
    throw std::invalid_argument("Video block budget must be 37..240 bytes");
  }
  if (encoded.empty()) return {};
  if (encoded.size() > std::numeric_limits<uint32_t>::max()) throw std::length_error("Frame too large");
  const size_t payload_capacity = block_max_bytes - kVideoHeaderSize;
  const auto count = (encoded.size() + payload_capacity - 1) / payload_capacity;
  if (count > std::numeric_limits<uint16_t>::max()) {
    throw std::length_error("Video frame exceeds fragment index range");
  }
  info.encoded_size = static_cast<uint32_t>(encoded.size());
  std::vector<VideoFragment> fragments(count);
  for (size_t i = 0; i < count; ++i) {
    auto & data = fragments[i];
    const auto size = std::min(payload_capacity, encoded.size() - i * payload_capacity);
    data.resize(kVideoHeaderSize + size, 0);
    data[0] = 'V';
    data[1] = 'S';
    data[2] = kVideoVersion;  // Application protocol version.
    data[3] = info.keyframe ? 1 : 0;
    put(data.data() + 4, info.frame_id, 4);
    put(data.data() + 8, info.timestamp_us, 8);
    put(data.data() + 16, info.encoded_size, 4);
    put(data.data() + 20, i, 2);
    put(data.data() + 22, count, 2);
    put(data.data() + 32, i * payload_capacity, 4);
    put(data.data() + 24, size, 2);
    put(data.data() + 26, info.width, 2);
    put(data.data() + 28, info.height, 2);
    data[30] = 1;  // Annex B HEVC / H.265.
    std::copy_n(encoded.data() + i * payload_capacity, size, data.data() + kVideoHeaderSize);
  }
  return fragments;
}

bool parse_video_fragment(const VideoFragment & data, VideoFragmentInfo & info)
{
  if (data.size() <= kVideoHeaderSize || data.size() > kVideoDataSize) return false;
  if (data[0] != 'V' || data[1] != 'S' || data[2] != kVideoVersion || data[30] != 1 ||
      (data[3] & ~1U) != 0 || data[31] != 0) return false;
  info.frame.frame_id = get(data.data() + 4, 4);
  info.frame.timestamp_us = get(data.data() + 8, 8);
  info.frame.encoded_size = get(data.data() + 16, 4);
  info.index = get(data.data() + 20, 2);
  info.count = get(data.data() + 22, 2);
  info.payload_size = get(data.data() + 24, 2);
  info.frame.width = get(data.data() + 26, 2);
  info.frame.height = get(data.data() + 28, 2);
  info.frame.keyframe = (data[3] & 1) != 0;
  if (info.frame.encoded_size == 0 || info.frame.width == 0 || info.frame.height == 0 ||
      info.count == 0 || info.index >= info.count) return false;
  info.offset = static_cast<uint32_t>(get(data.data() + 32, 4));
  return info.payload_size > 0 && info.payload_size <= kVideoPayloadSize &&
    data.size() == kVideoHeaderSize + info.payload_size &&
    uint64_t(info.offset) + info.payload_size <= info.frame.encoded_size &&
    info.count <= info.frame.encoded_size &&
    uint64_t(info.count) * kVideoPayloadSize >= info.frame.encoded_size &&
    (info.index != 0 || info.offset == 0) &&
    (info.index + 1 != info.count || uint64_t(info.offset) + info.payload_size == info.frame.encoded_size);
}
}  // namespace io
