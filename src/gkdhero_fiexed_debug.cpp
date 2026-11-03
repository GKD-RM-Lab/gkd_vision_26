#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gkdcontrol.hpp"
#include "io/video/video_streamer.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/center_aimer.hpp"
#include "tasks/auto_aim/center_shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{@config-path   | configs/gkdhero_fiexed.yaml | 位置参数，yaml配置文件路径 }";
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Recorder recorder;
  io::GKDControl gkdcontrol(config_path);
  io::VideoStreamer video_streamer(
    config_path, [&gkdcontrol](const io::VideoFragment & fragment) {
      return gkdcontrol.try_send_video(fragment);
    });
  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Color enemy_color = gkdcontrol.color_at(steady_clock::now());
  auto_aim::Tracker tracker(config_path, solver, enemy_color);
  auto_aim::Aimer aimer(config_path);
  auto_aim::CenterAimer center_aimer(config_path);
  auto_aim::CenterShooter center_shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  steady_clock::time_point t;
  int frame_count = 0;

  while (!exiter.exit()) {
    const auto loop_start = steady_clock::now();
    camera.read(img, t);
    if (img.empty()) continue;
    q = gkdcontrol.imu_at(t - 1ms);
    enemy_color = gkdcontrol.color_at(t - 1ms);

    solver.set_R_gimbal2world(q);

    const auto yolo_start = steady_clock::now();
    auto armors = detector.detect(img, frame_count);
    const auto tracker_start = steady_clock::now();
    auto targets = tracker.track(armors, t);
    const auto aimer_start = steady_clock::now();

    const auto armor_command = aimer.aim(targets, t, gkdcontrol.bullet_speed);
    auto command = center_aimer.aim(targets, t, armor_command, gkdcontrol.bullet_speed);
    const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    const auto fire =
      center_shooter.shoot(command, targets, t, gkdcontrol.bullet_speed, ypr[0]);
    command.shoot = fire.shoot;

    const auto finish = steady_clock::now();
    const auto total_ms = tools::delta_time(finish, loop_start) * 1e3;
    const auto fps = total_ms > 0.0 ? 1000.0 / total_ms : 0.0;

    tools::draw_text(
      img,
      fmt::format(
        "center command {}, yaw {:.2f}, pitch {:.2f}, shoot {}", command.control,
        command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 40}, {154, 50, 205});
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw {:.2f}, center err {:.2f}, armor err/tol {:.2f}/{:.2f}", ypr[0] * 57.3,
        fire.center_error * 57.3, fire.armor_error * 57.3, fire.armor_tolerance * 57.3),
      {10, 70}, {255, 255, 255});

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto state = target.ekf_x();
      for (const auto & xyza : target.armor_xyza_list()) {
        const auto image_points =
          solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }
      if (fire.predicted_armor.has_value()) {
        const auto & xyza = *fire.predicted_armor;
        const auto image_points =
          solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {255, 255, 0});
      }
      if (aimer.debug_aim_point.valid) {
        const auto & xyza = aimer.debug_aim_point.xyza;
        const auto image_points =
          solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 0, 255});
      }
      tools::logger()->info(
        "[HeroFixed][{}] center x {:.3f} m, y {:.3f} m, z {:.3f} m, yaw {:.3f} rad, "
        "flight {:.3f} s, shoot {}",
        frame_count, state[0], state[2], state[4], state[6], fire.flight_time, fire.shoot);
    }

    // Retain the full-size annotated buffer before the display-only 0.5x resize.
    // Only a shallow reference here; copy into the video queue after sending control.
    const cv::Mat video_frame = video_streamer.enabled() ? img : cv::Mat{};

    tools::logger()->info(
      "[{}] total: {:.1f}ms ({:.1f} FPS), yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: "
      "{:.1f}ms",
      frame_count, total_ms, fps, tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("gkdhero_fiexed", img);
    cv::waitKey(1);
    gkdcontrol.send(command);
    video_streamer.submit(video_frame, t);
    frame_count++;
  }

  return 0;
}
