#include "parameter_channel.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string_view>

#include "io/gkdcontrol/send_control.hpp"
#include "tools/logger.hpp"

namespace io
{
namespace
{
using Json = nlohmann::json;
using auto_aim::ParameterOperation;
constexpr std::size_t kMaxDatagram = 2048;

int64_t unix_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

bool valid_id(const Json & value)
{
  if (!value.is_string()) return false;
  const auto & id = value.get_ref<const std::string &>();
  return !id.empty() && id.size() <= 64 && std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '-' || c == '_';
  });
}

uint64_t unsigned_integer(const Json & value)
{
  if (!value.is_number_integer() || (value.is_number_integer() && !value.is_number_unsigned() &&
      value.get<int64_t>() < 0)) throw std::invalid_argument("invalid unsigned integer");
  return value.get<uint64_t>();
}

auto_aim::ParameterRequest parse_request(std::string_view bytes)
{
  // Small datagram + shallow schema bound parser work independently of the vision thread.
  const auto json = Json::parse(bytes, [](int depth, Json::parse_event_t, Json &) {
    if (depth > 4) throw std::invalid_argument("JSON nesting too deep");
    return true;
  });
  if (!json.is_object() || !json.contains("version") || unsigned_integer(json.at("version")) != 1 ||
      !json.contains("request_id") || !valid_id(json.at("request_id")))
    throw std::invalid_argument("invalid version/request_id");
  auto_aim::ParameterRequest request;
  request.request_id = json.at("request_id").get<std::string>();
  const auto operation = json.at("op").get<std::string>();
  std::vector<std::string> fields{"version", "request_id", "op"};
  if (operation == "get") request.operation = ParameterOperation::get;
  else {
    fields.insert(fields.end(), {"session_id", "base_revision", "expires_at_unix_ms"});
    if (!valid_id(json.at("session_id"))) throw std::invalid_argument("invalid session_id");
    request.session_id = json.at("session_id").get<std::string>();
    request.base_revision = unsigned_integer(json.at("base_revision"));
    const auto deadline = unsigned_integer(json.at("expires_at_unix_ms"));
    if (deadline > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
      throw std::invalid_argument("invalid deadline");
    request.expires_at_unix_ms = static_cast<int64_t>(deadline);
    if (operation == "bridge_notice") {
      request.operation = ParameterOperation::bridge_notice;
      fields.insert(fields.end(), {"sequence", "reason"});
      const auto sequence = unsigned_integer(json.at("sequence"));
      if (sequence > UINT16_MAX) throw std::invalid_argument("invalid sequence");
      request.sequence = static_cast<uint16_t>(sequence);
      request.notice_reason = json.at("reason").get<std::string>();
      if (request.notice_reason.empty() || request.notice_reason.size() > 48 ||
          !std::all_of(request.notice_reason.begin(), request.notice_reason.end(), [](unsigned char c) {
            return (c >= 'a' && c <= 'z') || c == '_';
          })) throw std::invalid_argument("invalid notice reason");
    } else if (operation == "set_ekf") {
      request.operation = ParameterOperation::set_ekf;
      fields.insert(fields.end(), {"process_v1", "process_v2"});
      if (!json.at("process_v1").is_number() || !json.at("process_v2").is_number())
        throw std::invalid_argument("v1/v2 must both be numbers");
      request.process_v1 = json.at("process_v1").get<double>();
      request.process_v2 = json.at("process_v2").get<double>();
    } else if (operation == "set_enemy_color" || operation == "apply") {
      request.operation = operation == "apply" ? ParameterOperation::apply : ParameterOperation::set_enemy_color;
      fields.insert(fields.end(), {"mode", "confirmation"});
      if (operation == "apply") {
        fields.insert(fields.end(), {"sequence", "update_ekf", "update_color", "process_v1", "process_v2"});
        const auto sequence = unsigned_integer(json.at("sequence"));
        if (!sequence || sequence > UINT16_MAX || !json.at("update_ekf").is_boolean() ||
            !json.at("update_color").is_boolean() || !json.at("process_v1").is_number() ||
            !json.at("process_v2").is_number()) throw std::invalid_argument("invalid apply fields");
        request.sequence = static_cast<uint16_t>(sequence);
        request.update_ekf = json.at("update_ekf"); request.update_color = json.at("update_color");
        if (!request.update_ekf && !request.update_color) throw std::invalid_argument("empty apply");
        request.process_v1 = json.at("process_v1"); request.process_v2 = json.at("process_v2");
      }
      const auto mode = json.at("mode").get<std::string>();
      if (mode == "auto") request.color_mode = auto_aim::EnemyColorMode::automatic;
      else if (mode == "red") request.color_mode = auto_aim::EnemyColorMode::red;
      else if (mode == "blue") request.color_mode = auto_aim::EnemyColorMode::blue;
      else throw std::invalid_argument("invalid color mode");
      const auto & confirmation = json.at("confirmation");
      if (!confirmation.is_object() || confirmation.size() != 2 ||
          confirmation.at("method") != "four_key_hold")
        throw std::invalid_argument("four_key_hold confirmation required");
      const auto held = unsigned_integer(confirmation.at("held_ms"));
      if (held > 10000) throw std::invalid_argument("invalid held_ms");
      request.confirmation_hold_ms = static_cast<int>(held);
    } else throw std::invalid_argument("unknown operation");
  }
  if (json.size() != fields.size()) throw std::invalid_argument("missing or extra fields");
  for (const auto & item : json.items())
    if (std::find(fields.begin(), fields.end(), item.key()) == fields.end())
      throw std::invalid_argument("unknown field");
  return request;
}

Json state_json(const auto_aim::RuntimeState & state, const auto_aim::RuntimeConfig & config)
{
  return {
    {"session_id", state.session_id}, {"revision", state.revision},
    {"process_v1", state.process_v1}, {"process_v2", state.process_v2},
    {"color_mode", auto_aim::color_mode_name(state.color_mode)},
    {"automatic_enemy_color", state.automatic_enemy == auto_aim::red ? "red" : "blue"},
    {"effective_enemy_color", state.effective_enemy == auto_aim::red ? "red" : "blue"},
    {"frame_seq", state.frame_seq}, {"capture_timestamp_us", state.capture_timestamp_us},
    {"tracking_state", state.tracking_state}, {"enabled", config.enabled},
    {"color_override_enabled", config.color_override_enabled},
    {"process_min_exclusive", config.process_min}, {"process_max_inclusive", config.process_max},
    {"max_request_ttl_ms", config.max_request_ttl_ms},
    {"color_confirmation_hold_ms", config.color_confirmation_hold_ms},
    {"last_request_id", state.last_request_id}, {"last_sequence", state.last_sequence},
    {"last_result", state.last_result}, {"last_reason", state.last_reason},
    {"last_duplicate", state.last_duplicate},
    {"last_bridge_notice", state.last_bridge_notice},
    {"session_started_at_unix_ms", state.session_started_at_unix_ms}};
}
}  // namespace

ParameterChannel::ParameterChannel(
  const auto_aim::RuntimeConfig & config, const auto_aim::RuntimeState & initial)
: config_(config), latest_state_(initial)
{
  if (!config_.enabled) return;
  socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0) throw std::runtime_error("parameter UDP socket: " + std::string(std::strerror(errno)));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(config_.request_port);
  if (bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    const auto message = std::string(std::strerror(errno));
    close(socket_);
    socket_ = -1;
    throw std::runtime_error("parameter UDP bind: " + message);
  }
  const int buffer_size = 8192;
  const int priority = 0;
  setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
  setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
  setsockopt(socket_, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
  try { worker_ = std::thread(&ParameterChannel::run, this); }
  catch (...) { close(socket_); socket_ = -1; throw; }
  tools::logger()->info(
    "[RuntimeParams] UDP 127.0.0.1:{} -> status/ack 127.0.0.1:{}, color override {}",
    config_.request_port, config_.status_port, config_.color_override_enabled ? "enabled" : "disabled");
}

ParameterChannel::~ParameterChannel()
{
  stop_ = true;
  if (worker_.joinable()) worker_.join();
  if (socket_ >= 0) close(socket_);
}

bool ParameterChannel::try_receive(auto_aim::ParameterRequest & request)
{
  if (socket_ < 0) return false;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  if (notice_) { request = std::move(*notice_); notice_.reset(); return true; }
  if (requests_.empty()) return false;
  request = std::move(requests_.front());
  requests_.pop_front();
  return true;
}

bool ParameterChannel::try_reply(const auto_aim::ParameterReply & reply)
{
  if (socket_ < 0) return false;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || replies_.size() >= config_.queue_capacity) return false;
  replies_.push_back(reply);
  return true;
}

void ParameterChannel::try_publish(const auto_aim::RuntimeState & state)
{
  if (socket_ < 0) return;
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) latest_state_ = state;
}

void ParameterChannel::run()
{
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10);
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  destination.sin_port = htons(config_.status_port);
  auto_aim::RuntimeState state;
  const auto send = [&](const Json & message) {
    if (control_send_active()) return;
    const auto bytes = message.dump();
    sendto(socket_, bytes.data(), bytes.size(), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
  };
  const auto status_period = std::chrono::duration<double>(1.0 / config_.status_hz);
  auto next_status = std::chrono::steady_clock::now();
  auto next_error = next_status;
  while (!stop_) {
    try {
      std::deque<auto_aim::ParameterReply> replies;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        replies.swap(replies_);
        state = latest_state_;
      }
      for (const auto & reply : replies) {
        send({{"version", 1}, {"type", "ack"}, {"request_id", reply.request_id},
          {"op", reply.operation == ParameterOperation::get ? "get" :
            reply.operation == ParameterOperation::set_ekf ? "set_ekf" :
            reply.operation == ParameterOperation::apply ? "apply" :
            reply.operation == ParameterOperation::bridge_notice ? "bridge_notice" : "set_enemy_color"},
          {"sequence", reply.sequence},
          {"result", reply.result}, {"reason", reply.reason}, {"duplicate", reply.duplicate},
          {"applied_revision", reply.applied_revision}, {"sent_at_unix_ms", unix_ms()},
          {"state", state_json(reply.state, config_)}});
        if (!reply.duplicate && reply.result == "applied")
          tools::logger()->info("[RuntimeParams] Applied {} revision {} v1={} v2={} color={}",
            reply.request_id, reply.applied_revision, reply.state.process_v1, reply.state.process_v2,
            auto_aim::color_mode_name(reply.state.color_mode));
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_status) {
        send({{"version", 1}, {"type", "status"}, {"sent_at_unix_ms", unix_ms()},
          {"state", state_json(state, config_)}});
        next_status = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(status_period);
      }
      pollfd descriptor{socket_, POLLIN, 0};
      if (poll(&descriptor, 1, 10) <= 0 || !(descriptor.revents & POLLIN)) continue;
      // Bound each receive batch even if a local client floods this optional interface.
      for (int i = 0; i < 8 && !stop_; ++i) {
        std::array<char, kMaxDatagram> buffer{};
        sockaddr_in source{}; socklen_t source_size = sizeof(source);
        const auto length = recvfrom(socket_, buffer.data(), buffer.size(), MSG_DONTWAIT | MSG_TRUNC,
          reinterpret_cast<sockaddr *>(&source), &source_size);
        if (length < 0) break;
        try {
          if (length == 0 || length > static_cast<ssize_t>(buffer.size()))
            throw std::invalid_argument("invalid datagram length");
          auto request = parse_request({buffer.data(), static_cast<std::size_t>(length)});
          if (request.operation == ParameterOperation::bridge_notice &&
              source.sin_port != htons(config_.status_port)) throw std::invalid_argument("untrusted notice source");
          request.received_at = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lock(mutex_);
          if (request.operation == ParameterOperation::bridge_notice) { notice_ = std::move(request); continue; }
          if (requests_.size() >= config_.queue_capacity) {
            if (request.operation == ParameterOperation::apply) {
              request.operation = ParameterOperation::bridge_notice;
              request.notice_reason = "vision_queue_full"; notice_ = std::move(request);
            }
            continue;  // no old command replacement
          }
          requests_.push_back(std::move(request));
        } catch (const std::exception &) {
          // Malformed packets never enter the vision queue; bound error response rate.
          if (std::chrono::steady_clock::now() >= next_error) {
            send({{"version", 1}, {"type", "error"}, {"reason", "malformed_request"}});
            next_error = std::chrono::steady_clock::now() + std::chrono::seconds(1);
          }
        }
      }
    } catch (const std::exception & error) {
      tools::logger()->warn("[RuntimeParams] Transport error: {}", error.what());
      poll(nullptr, 0, 10);
    }
  }
}
}  // namespace io
