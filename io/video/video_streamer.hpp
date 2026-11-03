#ifndef IO__VIDEO_STREAMER_HPP
#define IO__VIDEO_STREAMER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>

#include "video_packet.hpp"

namespace io
{
struct VideoStreamConfig
{
  bool enabled = false;
  int width = 256;
  int height = 192;
  int fps = 15;
  int bitrate = 48000;  // bits/s, leave headroom for metadata and packet padding.
  size_t queue_capacity = 1;
  int gop = 15;
  int packet_hz = 50;  // Official 0x0310 maximum, never exceed 50.
  int max_frame_age_ms = 500;
  double stats_interval_s = 2.0;
  uint16_t udp_port = 11454;  // Existing control bridge remains on 11451.

  static VideoStreamConfig load(const std::string & path);
  void validate() const;
};

struct VideoStreamStats
{
  uint64_t submitted = 0;
  uint64_t encoded = 0;
  uint64_t sent_frames = 0;  // All fragments accepted by local transport, not remote ACK.
  uint64_t sent_fragments = 0;
  uint64_t sent_bytes = 0;  // 0x0310 data bytes; excludes UDP/IP/serial outer framing.
  uint64_t queue_drops = 0;
  uint64_t transport_drops = 0;
  uint64_t errors = 0;
  uint64_t last_frame_bytes = 0;
  double last_encode_ms = 0;
  double send_fps = 0;
  double bandwidth_bytes_s = 0;
};

class VideoStreamer
{
public:
  // Sink must return immediately: false means control busy / transport backpressure.
  using Sink = std::function<bool(const VideoFragment &)>;
  VideoStreamer(const std::string & config_path, Sink sink);
  VideoStreamer(VideoStreamConfig config, Sink sink);
  ~VideoStreamer();
  VideoStreamer(const VideoStreamer &) = delete;
  VideoStreamer & operator=(const VideoStreamer &) = delete;

  // Rate-limited snapshot of the fully annotated full-size image. Never waits for queue space.
  bool submit(
    const cv::Mat & image,
    std::chrono::steady_clock::time_point capture_time = std::chrono::steady_clock::now()) noexcept;
  void set_enabled(bool enabled) noexcept;
  bool enabled() const noexcept;
  VideoStreamStats stats() const;

private:
  struct PendingFrame
  {
    cv::Mat image;
    std::chrono::steady_clock::time_point captured;
    uint64_t timestamp_us;
    uint32_t id;
  };

  VideoStreamConfig config_;
  Sink sink_;
  std::string config_path_;
  std::filesystem::file_time_type config_mtime_{};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> force_keyframe_{true};
  std::atomic<uint64_t> queue_drops_{0};
  std::atomic<uint64_t> submitted_{0};
  std::mutex queue_mutex_;
  std::condition_variable wake_;
  std::deque<PendingFrame> queue_;
  std::chrono::steady_clock::time_point last_submit_{};
  uint32_t next_frame_id_ = 0;
  mutable std::mutex stats_mutex_;
  VideoStreamStats stats_;
  std::thread worker_;  // Last member: all state exists before the worker starts.

  void start() noexcept;
  void run() noexcept;
  bool wait_until(std::chrono::steady_clock::time_point deadline);
  void reload_switch() noexcept;
  void report_stats();
};
}  // namespace io

#endif  // IO__VIDEO_STREAMER_HPP
