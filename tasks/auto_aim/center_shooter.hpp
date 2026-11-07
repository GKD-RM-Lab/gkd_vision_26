#ifndef AUTO_AIM__CENTER_SHOOTER_HPP
#define AUTO_AIM__CENTER_SHOOTER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <limits>
#include <list>
#include <optional>
#include <string>

#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct CenterFireDecision
{
  bool shoot = false;
  double flight_time = 0.0;
  double center_error = std::numeric_limits<double>::infinity();
  double armor_error = std::numeric_limits<double>::infinity();
  double armor_tolerance = 0.0;
  std::optional<Eigen::Vector4d> predicted_armor;
};

class CenterShooter
{
public:
  explicit CenterShooter(const std::string & config_path);

  CenterFireDecision shoot(
    const io::Command & command, const std::list<Target> & targets,
    std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    double gimbal_yaw) const;

private:
  double processing_delay(const Target & target) const;

  double yaw_offset_;
  double center_first_tolerance_;
  double center_second_tolerance_;
  double center_judge_distance_;
  double small_armor_width_;
  double big_armor_width_;
  double max_face_angle_;
  bool auto_fire_;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CENTER_SHOOTER_HPP
