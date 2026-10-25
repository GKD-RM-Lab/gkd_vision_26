#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/gkdcontrol.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

#include "tasks/omniperception/decider.hpp"
#include "io/usbcamera/usbcamera.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/gkdinfantry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;

  io::GKDControl gkdcontrol(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Color enemy_color = gkdcontrol.color_at(std::chrono::steady_clock::now());
  auto_aim::Tracker tracker(config_path, solver, enemy_color);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  //omniperception

  omniperception::Decider decider(config_path, enemy_color);
  io::USBCamera usbcam("video0", config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gkdcontrol.imu_at(t - 1ms);
    enemy_color = gkdcontrol.color_at(t - 1ms);

    // recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    decider.armor_filter(armors);
    decider.set_priority(armors);

    auto targets = tracker.track(armors, t);

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto state = target.ekf_x();
      tools::logger()->info(
        "[Infantry] Target state -> x {:.3f} m, y {:.3f} m, z {:.3f} m, yaw {:.3f} rad",
        state[0], state[2], state[4], state[6]);
    }

    io::Command command = {false,false ,0,0};
    
    if (tracker.state() == "lost")
      command = decider.decide(detector, gimbal_ypr, usbcam);
    else
      command = aimer.aim(targets, t, gkdcontrol.bullet_speed, gkdcontrol.shoot_mode);

    command.shoot = shooter.shoot(command, aimer, targets, gimbal_ypr);

    gkdcontrol.send(command);
  }

  return 0;
}
