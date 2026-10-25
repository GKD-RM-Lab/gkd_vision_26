#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gkdcontrol.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"


using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/gkdsentry.yaml | 位置参数，yaml 配置文件路径 }";

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

  // 使用传统识别器替代 YOLO 神经网络识别器
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Color enemy_color = gkdcontrol.color_at(std::chrono::steady_clock::now());
  auto_aim::Tracker tracker(config_path, solver, enemy_color);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    camera.read(img, t);
    if (img.empty()) continue;
    q = gkdcontrol.imu_at(t - 1ms);

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    auto detect_start = std::chrono::steady_clock::now();
    auto armors = detector.detect1(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, t, gkdcontrol.bullet_speed);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    auto finish = std::chrono::steady_clock::now();
    auto total_ms = tools::delta_time(finish, loop_start) * 1e3;
    auto fps = total_ms > 0.0 ? 1000.0 / total_ms : 0.0;

    tools::logger()->info(
      "[{}] total: {:.1f}ms ({:.1f} FPS), detect: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms",
      frame_count, total_ms, fps, tools::delta_time(tracker_start, detect_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command {}, yaw {:.2f}, pitch {:.2f}, shoot {}", command.control,
        command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 40}, {154, 50, 205});

    tools::draw_text(
      img, fmt::format("gimbal yaw {:.2f}, gimbal pitch {:.2f}", ypr[0] * 57.3 , ypr[1] * 57.3), {10, 70}, {255, 255, 255});

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto state = target.ekf_x();

      for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
        auto image_points =
          solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      const auto aim_point = aimer.debug_aim_point;
      if (aim_point.valid) {
        auto aim_points = solver.reproject_armor(
          aim_point.xyza.head<3>(), aim_point.xyza[3], target.armor_type, target.name);
        tools::draw_points(img, aim_points, {0, 0, 255});
      }

      tools::logger()->info(
        "[Infantry][{}] Target state -> x {:.3f} m, y {:.3f} m, z {:.3f} m, yaw {:.3f} rad",
        frame_count, state[0], state[2], state[4], state[6]);
    }

    tools::logger()->info(
      "[Infantry][{}] detect: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, detect_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("gkd_tradition", img);
    cv::waitKey(1);

    gkdcontrol.send(command);
    frame_count++;
  }

  return 0;
}
