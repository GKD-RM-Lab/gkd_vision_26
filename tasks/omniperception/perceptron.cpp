#include "perceptron.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(io::USBCamera * camera, const std::string & config_path)
: camera_(camera), detection_queue_(10)
{
  if (!camera_) throw std::invalid_argument("Perceptron requires one USB camera");

  yolo_ = std::make_unique<auto_aim::YOLO>(config_path, false);
  thread_ = std::thread([this] { infer(); });
  tools::logger()->info("Single-camera Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  stop_flag_ = true;
  if (thread_.joinable()) thread_.join();
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> results;
  DetectionResult result;
  while (detection_queue_.try_pop(result)) {
    results.push_back(std::move(result));
  }
  return results;
}

void Perceptron::infer()
{
  using namespace std::chrono_literals;
  try {
    while (!stop_flag_) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      if (!camera_->try_read(image, timestamp) || image.empty()) {
        std::this_thread::sleep_for(5ms);
        continue;
      }

      auto armors = yolo_->detect(image);
      if (stop_flag_ || armors.empty()) continue;

      // Preserve the legacy result type without producing direction/control information.
      DetectionResult result{std::move(armors), timestamp, 0, 0};
      detection_queue_.push(result);
    }
  } catch (const std::exception & error) {
    tools::logger()->error("[Perceptron] USB inference failed: {}", error.what());
    stop_flag_ = true;
  }
}
}  // namespace omniperception
