#ifndef IO__H265_ENCODER_HPP
#define IO__H265_ENCODER_HPP

#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace io
{
struct EncodedVideo
{
  std::vector<uint8_t> bytes;
  bool keyframe = false;
};

class H265Encoder
{
public:
  H265Encoder(int width, int height, int fps, int bitrate, int gop);
  ~H265Encoder();
  H265Encoder(const H265Encoder &) = delete;
  H265Encoder & operator=(const H265Encoder &) = delete;

  // The caller owns this encoder exclusively on the video worker thread.
  EncodedVideo encode(const cv::Mat & bgr, bool force_keyframe);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace io

#endif  // IO__H265_ENCODER_HPP
