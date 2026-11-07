#ifndef AUTO_AIM__CENTER_AIMER_HPP
#define AUTO_AIM__CENTER_AIMER_HPP

#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

class CenterAimer
{
public:
  explicit CenterAimer(const std::string & config_path);
  void reset_tracking();

  io::Command aim(
    const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp,
    const io::Command & armor_command, double bullet_speed);

private:
  double processing_delay(const Target & target) const;

  double yaw_offset_;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  std::optional<double> last_pitch_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CENTER_AIMER_HPP
