#include "gkdcontrol.hpp"

#include <thread>

#include "io/gkdcontrol/send_control.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace
{
const std::string kLoopbackIp = "127.0.0.1";
}  // namespace

namespace io
{
GKDControl::GKDControl(const std::string & config_path)
: mode(GKDMode::idle),
  shoot_mode(GKDShootMode::left_shoot),
  ft_angle(0.0),
  queue_(5000),
  color_queue_(5000)
{
  (void)config_path;
  tools::logger()->info("[GKDControl] Using fixed target IP {} for local communication.", kLoopbackIp);

  initialize_udp_transmission();
  initialize_udp_reception();

  tools::logger()->info("[GKDControl] Waiting for IMU samples...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  color_queue_.pop(color_ahead_);
  color_queue_.pop(color_behind_);
  tools::logger()->info("[GKDControl] Ready.");
}

Eigen::Quaterniond GKDControl::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = timestamp - t_a;

  double k = t_ac / t_ab;
  return q_a.slerp(k, q_b).normalized();
}

auto_aim::Color GKDControl::color_at(std::chrono::steady_clock::time_point timestamp)
{
  if (color_behind_.timestamp < timestamp) color_ahead_ = color_behind_;

  while (true) {
    color_queue_.pop(color_behind_);
    if (color_behind_.timestamp > timestamp) break;
    color_ahead_ = color_behind_;
  }

  return color_ahead_.enemy_color;
}

void GKDControl::send(Command command) const
{
  if(command.control == false) {
    return;
  }
  
  send_control(command.yaw, command.pitch, command.shoot);
}

void GKDControl::initialize_udp_reception()
{
  std::thread(&IO::Server_socket_interface::task, &socket_interface_).detach();

  std::thread([this]() {
    while (true) {
      ReceiveGimbalInfo current = socket_interface_.pkg;

      if (current.header == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      // 无奈之举
      current.yaw = -current.yaw;

      Eigen::Vector3d euler(current.yaw, current.pitch, current.roll);
      Eigen::Quaterniond q(tools::rotation_matrix(euler));
      const auto now = std::chrono::steady_clock::now();
      // Protocol: 1 means enemy is red, 0 means enemy is blue.
      const auto enemy_color =
        current.red ? auto_aim::Color::red : auto_aim::Color::blue;
      queue_.push({q.normalized(), now});
      color_queue_.push({enemy_color, now});

      // const double yaw = current.yaw;
      // const double pitch = current.pitch;
      // const bool red = current.red;
      // tools::logger()->info(
      //   "[GKDControl] Recv yaw {:.4f} rad ({:.2f} deg), pitch {:.4f} rad ({:.2f} deg), red: {}",
      //   yaw, yaw * RAD2DEG, pitch, pitch * RAD2DEG, red ? "true" : "false");

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }).detach();
}

void GKDControl::initialize_udp_transmission()
{
  init_send(kLoopbackIp);
}

}  // namespace io
