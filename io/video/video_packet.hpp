#ifndef IO__VIDEO_PACKET_HPP
#define IO__VIDEO_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace io
{
// 2027 CustomByteBlock.data budget: 300 B, of which 60 B is reserved (not sent).
constexpr uint16_t kVideoCommand = 0x0310;
constexpr size_t kCustomDataMaxBytes = 300;
constexpr size_t kOtherInfoReservedBytes = 60;
constexpr size_t kVideoDataSize = kCustomDataMaxBytes - kOtherInfoReservedBytes;
constexpr size_t kVideoHeaderSize = 36;
constexpr size_t kVideoPayloadSize = kVideoDataSize - kVideoHeaderSize;
constexpr uint8_t kVideoVersion = 2;
using VideoFragment = std::vector<uint8_t>;

struct VideoFrameInfo
{
  uint32_t frame_id = 0;
  uint64_t timestamp_us = 0;  // Unix capture time, microseconds.
  uint32_t encoded_size = 0;
  uint16_t width = 256;
  uint16_t height = 192;
  bool keyframe = false;
};

struct VideoFragmentInfo
{
  VideoFrameInfo frame;
  uint16_t index = 0;
  uint16_t count = 0;
  uint16_t payload_size = 0;
  uint32_t offset = 0;  // Byte offset in the encoded frame; independent of fragment size.
};

// Explicit little endian serialization; no compiler-dependent packed structs.
std::vector<VideoFragment> fragment_video(
  const std::vector<uint8_t> & encoded, VideoFrameInfo info,
  size_t block_max_bytes = kVideoDataSize);
bool parse_video_fragment(const VideoFragment & data, VideoFragmentInfo & info);
}  // namespace io

#endif  // IO__VIDEO_PACKET_HPP
