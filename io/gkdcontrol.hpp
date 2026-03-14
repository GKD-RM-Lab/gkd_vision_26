#ifndef IO__GKDCONTROL_HPP
#define IO__GKDCONTROL_HPP

#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <string_view>

#include "io/command.hpp"
#include "io/gkdcontrol/socket_interface.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
#include "io/gkdcontrol/send_control.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace io
{
enum class GKDMode
{
  idle = 0,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
inline constexpr std::array<std::string_view, 5> GKD_MODE_NAMES = {
  "idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum class GKDShootMode
{
  left_shoot = 0,
  right_shoot,
  both_shoot
};
inline constexpr std::array<std::string_view, 3> GKD_SHOOT_MODE_NAMES = {
  "left_shoot", "right_shoot", "both_shoot"};

class GKDControl
{
public:
  double bullet_speed = 23;
  GKDMode mode;
  GKDShootMode shoot_mode;
  double ft_angle;  //无人机专有

  GKDControl(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  auto_aim::Color color_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  struct Colors 
  {
    auto_aim::Color enemy_colors;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_;  // 必须在socket_之前初始化  
  tools::ThreadSafeQueue<Colors> color_queue;
  
  IMUData data_ahead_;
  IMUData data_behind_;

  Colors enemy_color_ahead_;
  Colors enemy_color_behind_;

  IO::Server_socket_interface socket_interface_;

  void initialize_udp_reception();
  void initialize_udp_transmission();
};

}  // namespace io

#endif