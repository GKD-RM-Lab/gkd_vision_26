#include "center_shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{

CenterShooter::CenterShooter(const std::string & config_path)
{
  const auto yaml = YAML::LoadFile(config_path);
  const auto fixed = yaml["fixed_shooter"];
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;
  auto_fire_ = yaml["auto_fire"].as<bool>();
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  center_first_tolerance_ = fixed["first_tolerance"].as<double>() / 57.3;
  center_second_tolerance_ = fixed["second_tolerance"].as<double>() / 57.3;
  center_judge_distance_ = fixed["judge_distance"].as<double>();
  small_armor_width_ = fixed["small_armor_width"].as<double>();
  big_armor_width_ = fixed["big_armor_width"].as<double>();
  max_face_angle_ = fixed["max_face_angle"].as<double>() / 57.3;
}

CenterFireDecision CenterShooter::shoot(
  const io::Command & command, const std::list<Target> & targets,
  std::chrono::steady_clock::time_point timestamp, double bullet_speed, double gimbal_yaw) const
{
  CenterFireDecision result;
  if (!command.control || targets.empty() || !auto_fire_ || bullet_speed <= 0) return result;

  auto target = targets.front();
  const auto state = target.ekf_x();
  const auto horizontal_distance = std::hypot(state[0], state[2]);
  const tools::Trajectory trajectory(bullet_speed, horizontal_distance, state[4]);
  if (trajectory.unsolvable) return result;
  result.flight_time = trajectory.fly_time;

  // Predict the vehicle translation and armor rotation from the image time to impact time.
  const auto impact_time =
    std::max(std::chrono::steady_clock::now(), timestamp) +
    std::chrono::microseconds(static_cast<int64_t>(
      (processing_delay(target) + result.flight_time) * 1e6));
  target.predict(impact_time);

  const auto impact_state = target.ekf_x();
  const auto center_yaw = std::atan2(impact_state[2], impact_state[0]);
  const auto center_command_yaw = tools::limit_rad(center_yaw + yaw_offset_);
  result.center_error = std::abs(tools::limit_rad(gimbal_yaw - center_command_yaw));

  const auto center_tolerance =
    std::hypot(impact_state[0], impact_state[2]) > center_judge_distance_
      ? center_second_tolerance_
      : center_first_tolerance_;
  if (result.center_error > center_tolerance) return result;

  const auto armor_width =
    target.armor_type == ArmorType::big ? big_armor_width_ : small_armor_width_;
  for (const auto & armor : target.armor_xyza_list()) {
    const auto armor_yaw = std::atan2(armor[1], armor[0]);
    const auto face_error = std::abs(tools::limit_rad(armor[3] - center_yaw));

    // Do not fire at an armor occluded by the vehicle body.
    if (face_error >= max_face_angle_) continue;

    const auto distance = std::hypot(armor[0], armor[1]);
    const auto projected_half_width = armor_width * 0.5 * std::cos(face_error);
    const auto armor_tolerance = std::atan2(projected_half_width, distance);
    const auto armor_command_yaw = tools::limit_rad(armor_yaw + yaw_offset_);
    const auto armor_error = std::abs(tools::limit_rad(gimbal_yaw - armor_command_yaw));

    if (armor_error < result.armor_error) {
      result.armor_error = armor_error;
      result.armor_tolerance = armor_tolerance;
      result.predicted_armor = armor;
    }
  }

  result.shoot =
    result.predicted_armor.has_value() && result.armor_error <= result.armor_tolerance;
  return result;
}

double CenterShooter::processing_delay(const Target & target) const
{
  return std::abs(target.ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_
                                                       : low_speed_delay_time_;
}

}  // namespace auto_aim
