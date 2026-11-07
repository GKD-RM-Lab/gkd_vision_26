#ifndef AUTO_AIM__RUNTIME_PARAMS_HPP
#define AUTO_AIM__RUNTIME_PARAMS_HPP

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
enum class EnemyColorMode { automatic, red, blue };
// Legacy operations remain local-only; compact MQTT uses one atomic apply operation.
enum class ParameterOperation { get, set_ekf, set_enemy_color, apply, bridge_notice };
const char * color_mode_name(EnemyColorMode mode);

struct RuntimeConfig
{
  bool enabled = false;
  bool color_override_enabled = false;
  bool show_overlay = true;
  uint16_t request_port = 11455;
  uint16_t status_port = 11456;
  std::size_t queue_capacity = 8;
  int max_request_ttl_ms = 2000;
  int max_queue_age_ms = 500;
  int color_confirmation_hold_ms = 500;
  double status_hz = 1.0;
  double process_min = 0.0;  // exclusive lower bound
  double process_max = 1e6;  // numerical input guard, not a tuning recommendation
};

struct ParameterRequest
{
  std::string request_id;
  ParameterOperation operation = ParameterOperation::get;
  std::string session_id;
  uint64_t base_revision = 0;
  int64_t expires_at_unix_ms = 0;
  double process_v1 = 0;
  double process_v2 = 0;
  EnemyColorMode color_mode = EnemyColorMode::automatic;
  int confirmation_hold_ms = 0;
  std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now();
  uint16_t sequence = 0;
  bool update_ekf = false;
  bool update_color = false;
  std::string notice_reason;
};

struct RuntimeState
{
  std::string session_id;
  uint64_t revision = 0;
  double process_v1 = 0;
  double process_v2 = 0;
  EnemyColorMode color_mode = EnemyColorMode::automatic;
  Color automatic_enemy = red;
  Color effective_enemy = red;
  uint64_t frame_seq = 0;  // camera-loop sequence, not encoded-video frame_id
  int64_t capture_timestamp_us = 0;
  std::string tracking_state = "lost";
  std::string last_request_id;
  uint16_t last_sequence = 0;
  std::string last_result;
  std::string last_reason;
  bool last_duplicate = false;
  bool last_bridge_notice = false;
  int64_t session_started_at_unix_ms = 0;
};

struct ParameterReply
{
  std::string request_id;
  std::string result;
  std::string reason;
  bool duplicate = false;
  uint64_t applied_revision = 0;
  RuntimeState state;
  ParameterOperation operation = ParameterOperation::get;
  uint16_t sequence = 0;
};

// Owned exclusively by the vision thread. Network threads only transfer typed messages.
class RuntimeParams
{
public:
  RuntimeParams(const std::string & config_path, Color automatic_enemy);
  const RuntimeConfig & config() const { return config_; }
  const RuntimeState & state() const { return state_; }
  void begin_frame(Color automatic_enemy, uint64_t frame_seq, int64_t capture_timestamp_us);
  void set_tracking_state(const std::string & state) { state_.tracking_state = state; }
  ParameterReply apply(const ParameterRequest & request, int64_t now_unix_ms);
  void draw_overlay(cv::Mat & image) const;

private:
  struct CachedRequest
  {
    ParameterRequest request;
    uint64_t applied_revision;
  };
  RuntimeConfig config_;
  RuntimeState state_;
  std::deque<CachedRequest> applied_requests_;  // bounded replay window, current process only
  void resolve_enemy();
};
}  // namespace auto_aim
#endif
