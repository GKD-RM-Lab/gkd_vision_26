#include "video_streamer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#ifdef SP_VISION_ENABLE_VIDEO
#include <opencv2/imgproc.hpp>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "h265_encoder.hpp"
#endif

namespace io
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

VideoStreamConfig VideoStreamConfig::load(const std::string & path)
{
  VideoStreamConfig result;
  const auto node = YAML::LoadFile(path)["video"];
  if (!node) return result;
  result.enabled = node["enabled"].as<bool>(result.enabled);
  result.width = node["width"].as<int>(result.width);
  result.height = node["height"].as<int>(result.height);
  result.fps = node["fps"].as<int>(result.fps);
  result.bitrate = node["bitrate"].as<int>(result.bitrate);
  const int capacity = node["queue_capacity"].as<int>(1);
  if (capacity < 1 || capacity > 8) throw std::invalid_argument("video.queue_capacity must be 1..8");
  result.queue_capacity = static_cast<size_t>(capacity);
  result.gop = node["gop"].as<int>(result.gop);
  result.packet_hz = node["packet_hz"].as<int>(result.packet_hz);
  result.video_block_max_bytes = node["video_block_max_bytes"].as<int>(result.video_block_max_bytes);
  result.other_info_reserved_bytes = node["other_info_reserved_bytes"].as<int>(result.other_info_reserved_bytes);
  result.max_frame_age_ms = node["max_frame_age_ms"].as<int>(result.max_frame_age_ms);
  result.stats_interval_s = node["stats_interval_s"].as<double>(result.stats_interval_s);
  const int port = node["udp_port"].as<int>(result.udp_port);
  if (port < 1 || port > 65535) throw std::invalid_argument("Invalid video.udp_port");
  result.udp_port = static_cast<uint16_t>(port);
  result.validate();
  return result;
}

void VideoStreamConfig::validate() const
{
  if (width < 16 || height < 16 || width > 1920 || height > 1080 || width % 2 || height % 2 ||
      fps < 1 || fps > 60 || bitrate < 1000 || bitrate > 1000000 || queue_capacity < 1 ||
      queue_capacity > 8 || gop < 1 || gop > 600 || packet_hz < 1 || packet_hz > 50 ||
      video_block_max_bytes <= static_cast<int>(kVideoHeaderSize) ||
      video_block_max_bytes > static_cast<int>(kVideoDataSize) || other_info_reserved_bytes < 60 ||
      other_info_reserved_bytes > 263 || video_block_max_bytes + other_info_reserved_bytes > 300 ||
      max_frame_age_ms < 20 || max_frame_age_ms > 2000 ||
      !std::isfinite(stats_interval_s) || stats_interval_s < 0.1 || stats_interval_s > 3600 ||
      udp_port == 0 || udp_port == 11451 || udp_port == 11452 || udp_port == 11453) {
    throw std::invalid_argument("Invalid video configuration (see VIDEO_STREAMING_CHANGES.md)");
  }
  // Even before padding, coding above this budget cannot fit in the official link.
  if (bitrate > static_cast<int>((video_block_max_bytes - kVideoHeaderSize) * packet_hz * 8)) {
    throw std::invalid_argument("video.bitrate exceeds the configured 0x0310 payload budget");
  }
}

VideoStreamer::VideoStreamer(const std::string & config_path, Sink sink)
: sink_(std::move(sink)), config_path_(config_path)
{
  try {
    config_ = VideoStreamConfig::load(config_path);
    std::error_code error;
    config_mtime_ = std::filesystem::last_write_time(config_path_, error);
    start();
  } catch (const std::exception & error) {
    tools::logger()->warn("[VideoStreamer] Disabled: {}", error.what());
  }
}

VideoStreamer::VideoStreamer(VideoStreamConfig config, Sink sink)
: config_(config), sink_(std::move(sink))
{
  // Direct construction is also used by standalone tests; invalid config is a caller error.
  config_.validate();
  start();
}

void VideoStreamer::start() noexcept
{
#ifdef SP_VISION_ENABLE_VIDEO
  try {
    if (!sink_) throw std::invalid_argument("Video sink is empty");
    enabled_ = config_.enabled;
    worker_ = std::thread(&VideoStreamer::run, this);
  } catch (const std::exception & error) {
    enabled_ = false;
    tools::logger()->warn("[VideoStreamer] Cannot start video worker: {}", error.what());
  }
#else
  if (config_.enabled) tools::logger()->warn("[VideoStreamer] Built without H.265 support");
#endif
}

VideoStreamer::~VideoStreamer()
{
  stopping_ = true;
  wake_.notify_all();
  if (worker_.joinable()) worker_.join();
}

bool VideoStreamer::submit(const cv::Mat & image, Clock::time_point capture_time) noexcept
{
  if (!enabled_ || stopping_ || image.empty() || image.type() != CV_8UC3) return false;
  try {
    std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      ++queue_drops_;
      return false;
    }
    const auto now = Clock::now();
    if (now - last_submit_ < std::chrono::duration<double>(1.0 / config_.fps)) return false;
    if (!enabled_) return false;
    // Own the snapshot before the next loop reuses the camera/annotation buffer.
    PendingFrame frame;
    frame.image = image.clone();
    frame.captured = std::min(capture_time, now);
    const auto unix_capture = std::chrono::system_clock::now().time_since_epoch() -
      std::chrono::duration_cast<std::chrono::system_clock::duration>(now - frame.captured);
    frame.timestamp_us =
      std::chrono::duration_cast<std::chrono::microseconds>(unix_capture).count();
    while (queue_.size() >= config_.queue_capacity) {
      queue_.pop_front();
      ++queue_drops_;
    }
    queue_.push_back(std::move(frame));
    last_submit_ = now;
    lock.unlock();
    ++submitted_;
    wake_.notify_one();
    return true;
  } catch (...) {
    ++queue_drops_;
    return false;  // Allocation failure must not escape into auto aim.
  }
}

void VideoStreamer::set_enabled(bool enabled) noexcept
{
#ifndef SP_VISION_ENABLE_VIDEO
  enabled = false;
#endif
  enabled_ = enabled;
  force_keyframe_ = true;
  wake_.notify_all();
}

bool VideoStreamer::enabled() const noexcept { return enabled_; }

VideoStreamStats VideoStreamer::stats() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  auto result = stats_;
  result.queue_drops = queue_drops_;
  result.submitted = submitted_;
  return result;
}

bool VideoStreamer::wait_until(Clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(queue_mutex_);
  wake_.wait_until(lock, deadline, [this] { return stopping_ || !enabled_; });
  return !stopping_ && enabled_;
}

void VideoStreamer::reload_switch() noexcept
{
  if (config_path_.empty()) return;
  try {
    std::error_code error;
    const auto mtime = std::filesystem::last_write_time(config_path_, error);
    if (error || mtime == config_mtime_) return;
    config_mtime_ = mtime;
    // Only the switch is hot-reloaded. Encoder/transport parameters are startup settings.
    const auto video = YAML::LoadFile(config_path_)["video"];
    const bool requested = video ? video["enabled"].as<bool>(false) : false;
    if (requested != enabled_) {
      set_enabled(requested);
      tools::logger()->info("[VideoStreamer] YAML switch: {}", requested ? "on" : "off");
    }
  } catch (const std::exception & error) {
    set_enabled(false);
    tools::logger()->warn("[VideoStreamer] YAML reload failed, video off: {}", error.what());
  }
}

void VideoStreamer::report_stats()
{
  const auto snapshot = stats();
  tools::logger()->info(
    "[VideoStreamer] encode {:.2f}ms, size {}B, send {:.2f} FPS, {:.0f} B/s, "
    "queue drops {}, transport drops {}, errors {}",
    snapshot.last_encode_ms, snapshot.last_frame_bytes, snapshot.send_fps,
    snapshot.bandwidth_bytes_s, snapshot.queue_drops, snapshot.transport_drops, snapshot.errors);
}

void VideoStreamer::run() noexcept
{
#ifdef SP_VISION_ENABLE_VIDEO
  // Linux nice applies to this thread only. Failure simply leaves default scheduling.
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10);
  std::unique_ptr<H265Encoder> encoder;
  auto next_packet = Clock::now();
  auto next_encode = Clock::now();
  auto next_reload = Clock::now();
  auto last_report = Clock::now();
  uint64_t previous_sent = 0;
  uint64_t previous_bytes = 0;
  const auto packet_period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / config_.packet_hz));
  const auto frame_period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / config_.fps));

  while (!stopping_) {
    try {
      auto now = Clock::now();
      if (now >= next_reload) {
        reload_switch();
        next_reload = now + 1s;
      }
      if (std::chrono::duration<double>(now - last_report).count() >= config_.stats_interval_s) {
        bool should_report = enabled_;
        {
          std::lock_guard<std::mutex> lock(stats_mutex_);
          should_report = should_report || stats_.sent_frames != previous_sent;
          const double elapsed = std::chrono::duration<double>(now - last_report).count();
          stats_.send_fps = (stats_.sent_frames - previous_sent) / elapsed;
          stats_.bandwidth_bytes_s = (stats_.sent_bytes - previous_bytes) / elapsed;
          previous_sent = stats_.sent_frames;
          previous_bytes = stats_.sent_bytes;
        }
        last_report = now;
        if (should_report) report_stats();
      }

      PendingFrame pending;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (!enabled_) {
          queue_drops_ += queue_.size();
          queue_.clear();
          encoder.reset();
          wake_.wait_for(lock, 100ms, [this] { return stopping_ || enabled_; });
          continue;
        }
        if (queue_.empty()) {
          wake_.wait_for(lock, 50ms, [this] { return stopping_ || !enabled_ || !queue_.empty(); });
          continue;
        }
        if (now < next_encode) {
          wake_.wait_until(lock, std::min(next_encode, now + 50ms));
          continue;
        }
        // Take the newest snapshot even when a larger queue was configured.
        pending = std::move(queue_.back());
        queue_drops_ += queue_.size() - 1;
        queue_.clear();
      }
      const auto deadline = pending.captured + std::chrono::milliseconds(config_.max_frame_age_ms);
      if (Clock::now() >= deadline) {
        ++queue_drops_;
        continue;  // Not encoded: no reference-frame discontinuity.
      }
      if (!encoder) {
        encoder = std::make_unique<H265Encoder>(
          config_.width, config_.height, config_.fps, config_.bitrate, config_.gop);
        force_keyframe_ = true;
      }
      cv::Mat resized;
      cv::resize(pending.image, resized, cv::Size(config_.width, config_.height), 0, 0, cv::INTER_AREA);
      const auto encode_start = Clock::now();
      next_encode = encode_start + frame_period;
      const auto encoded = encoder->encode(resized, force_keyframe_.exchange(false));
      const double encode_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - encode_start).count();
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.encoded;
        stats_.last_encode_ms = encode_ms;
        stats_.last_frame_bytes = encoded.bytes.size();
      }
      VideoFrameInfo info;
      info.frame_id = next_frame_id_++;  // Only encoded frames consume reference-chain IDs.
      info.timestamp_us = pending.timestamp_us;
      info.width = static_cast<uint16_t>(config_.width);
      info.height = static_cast<uint16_t>(config_.height);
      info.keyframe = encoded.keyframe;
      const auto fragments = fragment_video(encoded.bytes, info, config_.video_block_max_bytes);
      bool sent = true;
      // Reject a frame that cannot finish before its deadline, rather than queueing stale fragments.
      const auto first_slot = std::max(Clock::now(), next_packet);
      if (first_slot + packet_period * (fragments.size() - 1) >= deadline) sent = false;
      for (const auto & fragment : fragments) {
        if (!sent) break;
        while (true) {
          if (!wait_until(next_packet) || Clock::now() >= deadline) {
            sent = false;
            break;
          }
          if (sink_(fragment)) {
            next_packet = Clock::now() + packet_period;  // No catch-up bursts.
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.sent_fragments;
            stats_.sent_bytes += fragment.size();
            break;
          }
          // Nonblocking transport declined; yield to control, bounded by frame deadline.
          next_packet = Clock::now() + 1ms;
        }
      }
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (sent) ++stats_.sent_frames;
        else ++stats_.transport_drops;
      }
      if (!sent) force_keyframe_ = true;  // Next transmitted frame must recover decoder references.
    } catch (const std::exception & error) {
      encoder.reset();
      force_keyframe_ = true;
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.errors;
      }
      // Fail closed: encoder/transport faults never terminate the auto-aim loop.
      set_enabled(false);
      tools::logger()->warn("[VideoStreamer] Video off after worker error: {}", error.what());
    }
  }
#endif
}
}  // namespace io
