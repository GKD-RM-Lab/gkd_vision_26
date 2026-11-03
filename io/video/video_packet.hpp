#ifndef IO__VIDEO_PACKET_HPP
#define IO__VIDEO_PACKET_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace io
{
// RoboMaster 2026 V1.3.0: 0x0310 has a fixed 300-byte data segment.
constexpr uint16_t kVideoCommand = 0x0310;
constexpr size_t kVideoDataSize = 300;
constexpr size_t kVideoHeaderSize = 32;
constexpr size_t kVideoPayloadSize = kVideoDataSize - kVideoHeaderSize;
using VideoFragment = std::array<uint8_t, kVideoDataSize>;

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
};

// Explicit little endian serialization; no compiler-dependent packed structs.
std::vector<VideoFragment> fragment_video(
  const std::vector<uint8_t> & encoded, VideoFrameInfo info);
bool parse_video_fragment(const VideoFragment & data, VideoFragmentInfo & info);
}  // namespace io

#endif  // IO__VIDEO_PACKET_HPP
