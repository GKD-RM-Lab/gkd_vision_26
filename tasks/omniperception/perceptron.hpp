#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{

class Perceptron
{
public:
  // The caller owns the camera and must keep it alive until this object is destroyed.
  Perceptron(io::USBCamera * camera, const std::string & config_path);

  ~Perceptron();

  // Observation-only results: legacy direction fields are zero, not estimated angles.
  std::vector<DetectionResult> get_detection_queue();

private:
  io::USBCamera * camera_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;
  std::unique_ptr<auto_aim::YOLO> yolo_;
  std::atomic<bool> stop_flag_{false};
  std::thread thread_;

  void infer();
};

}  // namespace omniperception
#endif
