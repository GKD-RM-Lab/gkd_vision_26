#include "runtime_params.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>

namespace auto_aim
{
namespace
{
bool same_request(const ParameterRequest & a, const ParameterRequest & b)
{
  return a.operation == b.operation && a.session_id == b.session_id &&
         a.base_revision == b.base_revision && a.expires_at_unix_ms == b.expires_at_unix_ms &&
         a.process_v1 == b.process_v1 && a.process_v2 == b.process_v2 &&
         a.color_mode == b.color_mode && a.confirmation_hold_ms == b.confirmation_hold_ms &&
         a.sequence == b.sequence && a.update_ekf == b.update_ekf && a.update_color == b.update_color;
}

std::string new_session_id()
{
  std::random_device random;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) stream << std::setw(8) << static_cast<uint32_t>(random());
  return stream.str();
}
}  // namespace

const char * color_mode_name(EnemyColorMode mode)
{
  switch (mode) {
    case EnemyColorMode::automatic: return "auto";
    case EnemyColorMode::red: return "red";
    case EnemyColorMode::blue: return "blue";
  }
  return "invalid";
}

RuntimeParams::RuntimeParams(const std::string & config_path, Color automatic_enemy)
{
  const auto yaml = YAML::LoadFile(config_path);
  state_.session_id = new_session_id();
  state_.session_started_at_unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  state_.process_v1 = yaml["process_v1"].as<double>(100.0);
  state_.process_v2 = yaml["process_v2"].as<double>(400.0);
  state_.automatic_enemy = automatic_enemy;
  const auto node = yaml["runtime_params"];
  if (node) {
    config_.enabled = node["enabled"].as<bool>(false);
    config_.color_override_enabled = node["color_override_enabled"].as<bool>(false);
    config_.show_overlay = node["show_overlay"].as<bool>(true);
    const int request_port = node["request_port"].as<int>(11455);
    const int status_port = node["status_port"].as<int>(11456);
    const int video_port = yaml["video"] ? yaml["video"]["udp_port"].as<int>(11454) : 11454;
    const auto valid_port = [&](int port) {
      return port > 0 && port <= 65535 && !(port >= 11451 && port <= 11454) &&
             port != video_port;
    };
    if (!valid_port(request_port) || !valid_port(status_port) || request_port == status_port)
      throw std::invalid_argument("runtime_params: invalid/conflicting UDP ports");
    config_.request_port = static_cast<uint16_t>(request_port);
    config_.status_port = static_cast<uint16_t>(status_port);
    const int capacity = node["queue_capacity"].as<int>(8);
    config_.max_request_ttl_ms = node["max_request_ttl_ms"].as<int>(2000);
    config_.max_queue_age_ms = node["max_queue_age_ms"].as<int>(500);
    config_.color_confirmation_hold_ms = node["color_confirmation_hold_ms"].as<int>(500);
    config_.status_hz = node["status_hz"].as<double>(1.0);
    config_.process_min = node["process_min_exclusive"].as<double>(0.0);
    config_.process_max = node["process_max_inclusive"].as<double>(1e6);
    if (capacity < 1 || capacity > 32 || config_.max_request_ttl_ms < 100 ||
        config_.max_request_ttl_ms > 10000 || config_.max_queue_age_ms < 1 ||
        config_.max_queue_age_ms > config_.max_request_ttl_ms ||
        config_.color_confirmation_hold_ms < 1 || config_.color_confirmation_hold_ms > 10000 ||
        !std::isfinite(config_.status_hz) || config_.status_hz < 0.1 || config_.status_hz > 10 ||
        !std::isfinite(config_.process_min) || config_.process_min < 0 ||
        !std::isfinite(config_.process_max) || config_.process_max <= config_.process_min)
      throw std::invalid_argument("runtime_params: invalid queue/timing/parameter limits");
    config_.queue_capacity = static_cast<std::size_t>(capacity);
  }
  const auto valid_initial = [&](double value) {
    return std::isfinite(value) && value > config_.process_min && value <= config_.process_max;
  };
  if (config_.enabled && (!valid_initial(state_.process_v1) || !valid_initial(state_.process_v2)))
    throw std::invalid_argument("runtime_params: startup v1/v2 outside configured limits");
  resolve_enemy();
}

void RuntimeParams::resolve_enemy()
{
  state_.effective_enemy = state_.color_mode == EnemyColorMode::automatic ? state_.automatic_enemy :
                           state_.color_mode == EnemyColorMode::red ? red : blue;
}

void RuntimeParams::begin_frame(Color automatic_enemy, uint64_t frame_seq, int64_t timestamp_us)
{
  state_.automatic_enemy = automatic_enemy;
  state_.frame_seq = frame_seq;
  state_.capture_timestamp_us = timestamp_us;
  resolve_enemy();
}

ParameterReply RuntimeParams::apply(const ParameterRequest & request, int64_t now_unix_ms)
{
  ParameterReply reply{request.request_id, "rejected", "", false, 0, state_};
  reply.operation = request.operation;
  reply.sequence = request.sequence;
  const auto finish = [&]() {
    if (request.operation != ParameterOperation::get) {
      state_.last_request_id = request.request_id;
      state_.last_sequence = request.sequence;
      state_.last_result = reply.result;
      state_.last_reason = reply.reason;
      state_.last_duplicate = reply.duplicate;
      state_.last_bridge_notice = request.operation == ParameterOperation::bridge_notice;
    }
    reply.state = state_;
    return reply;
  };
  const auto reject = [&](const char * reason) {
    reply.reason = reason;
    return finish();
  };
  if (!config_.enabled) return reject("disabled");
  if (request.operation == ParameterOperation::get) {
    reply.result = "snapshot";
    return reply;
  }
  if (request.operation == ParameterOperation::bridge_notice && request.session_id != state_.session_id)
    return reply;
  if (request.session_id != state_.session_id) return reject("session_mismatch");
  if (request.operation == ParameterOperation::bridge_notice) {
    // A late bridge diagnostic must not overwrite a new process's HUD.
    if (request.expires_at_unix_ms <= now_unix_ms ||
        std::chrono::steady_clock::now() - request.received_at >
          std::chrono::milliseconds(config_.max_queue_age_ms)) return reply;
    return reject(request.notice_reason.c_str());
  }
  for (const auto & cached : applied_requests_) {
    if (cached.request.request_id != request.request_id) continue;
    if (!same_request(cached.request, request)) return reject("request_id_conflict");
    reply.result = "applied";
    reply.duplicate = true;
    reply.applied_revision = cached.applied_revision;
    return finish();  // exact retries only acknowledge; never execute twice
  }
  if (request.expires_at_unix_ms <= now_unix_ms ||
      request.expires_at_unix_ms > now_unix_ms + config_.max_request_ttl_ms)
    return reject("expired_or_invalid_deadline");
  const auto age = std::chrono::steady_clock::now() - request.received_at;
  if (age > std::chrono::milliseconds(config_.max_queue_age_ms)) return reject("queue_expired");
  if (request.base_revision != state_.revision) return reject("revision_conflict");
  // Wire v1 uses uint32 for the expected revision. Never truncate or wrap a session.
  if (state_.revision >= std::numeric_limits<uint32_t>::max()) return reject("revision_exhausted");

  const bool ekf = request.operation == ParameterOperation::set_ekf ||
    (request.operation == ParameterOperation::apply && request.update_ekf);
  const bool color = request.operation == ParameterOperation::set_enemy_color ||
    (request.operation == ParameterOperation::apply && request.update_color);
  if (!ekf && !color) return reject("unknown_operation");
  if (request.operation == ParameterOperation::apply && request.sequence == 0)
    return reject("invalid_sequence");
  // Validate every selected field before changing any part of the transaction.
  if (ekf) {
    const auto valid = [&](double value) {
      return std::isfinite(value) && value > config_.process_min && value <= config_.process_max;
    };
    if (!valid(request.process_v1) || !valid(request.process_v2)) return reject("invalid_v1_v2");
  }
  if (color) {
    if (!config_.color_override_enabled) return reject("color_override_disabled");
    if (request.color_mode != EnemyColorMode::automatic && request.color_mode != EnemyColorMode::red &&
        request.color_mode != EnemyColorMode::blue) return reject("invalid_color_mode");
    if (request.confirmation_hold_ms < config_.color_confirmation_hold_ms ||
        request.confirmation_hold_ms > 10000) return reject("color_confirmation_required");
  }
  if (ekf) { state_.process_v1 = request.process_v1; state_.process_v2 = request.process_v2; }
  if (color) { state_.color_mode = request.color_mode; resolve_enemy(); }
  ++state_.revision;
  applied_requests_.push_back({request, state_.revision});
  if (applied_requests_.size() > 128) applied_requests_.pop_front();
  reply.result = "applied";
  reply.applied_revision = state_.revision;
  return finish();
}

void RuntimeParams::draw_overlay(cv::Mat & image) const
{
  if (!config_.enabled || !config_.show_overlay || image.empty()) return;
  std::ostringstream first, values;
  first << "CMD ";
  if (state_.last_result.empty()) first << "-- READY";
  else first << '#' << state_.last_sequence << ' ' <<
    (state_.last_result == "applied" ? "OK" : "ERR") << (state_.last_duplicate ? " DUP" : "");
  first << " r" << state_.revision;
  values << "V1=" << std::setprecision(7) << state_.process_v1 << " V2=" << state_.process_v2;
  const auto color = std::string("EN=") + color_mode_name(state_.color_mode) + ">" +
    (state_.effective_enemy == red ? "RED " : "BLUE ") + state_.tracking_state;
  // Draw on the original full image; remain readable after 256x192 downscaling.
  const double scale = image.cols / 256.0;
  const int thickness = std::max(1, static_cast<int>(scale));
  const auto draw = [&](const std::string & text, int row, const cv::Scalar & ink) {
    int baseline = 0;
    auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.34 * scale, thickness, &baseline);
    const double font = 0.34 * scale * std::min(1.0, (image.cols - 6 * scale) / std::max(1, size.width));
    const cv::Point origin{static_cast<int>(3 * scale), static_cast<int>((11 + row * 13) * scale)};
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font,
      {0, 0, 0}, thickness * 3, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font, ink, thickness, cv::LINE_AA);
  };
  draw(first.str(), 0, state_.last_result == "rejected" ? cv::Scalar(0, 80, 255) : cv::Scalar(80, 255, 80));
  draw(values.str(), 1, {255, 255, 255});
  draw(color, 2, {255, 255, 255});
  if (!state_.last_reason.empty()) draw(state_.last_reason, 3, {0, 180, 255});
}
}  // namespace auto_aim
