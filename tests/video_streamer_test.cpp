#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <thread>

#include "io/gkdcontrol/send_control.hpp"
#include "io/gkdcontrol/video_sender.hpp"
#include "io/video/video_packet.hpp"
#include "io/video/video_streamer.hpp"

#ifdef SP_VISION_ENABLE_VIDEO
#include "io/video/h265_encoder.hpp"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}
#endif

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename Predicate>
void eventually(Predicate predicate, const std::string & message, int timeout_ms = 3000)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!predicate()) {
    require(Clock::now() < deadline, message);
    std::this_thread::sleep_for(2ms);
  }
}

void test_packets()
{
  require(io::fragment_video({}, {}).empty(), "Empty frame should not emit a packet");
  for (const size_t size : {size_t(1), size_t(268), size_t(269), size_t(536), size_t(65537)}) {
    std::vector<uint8_t> source(size);
    for (size_t i = 0; i < size; ++i) source[i] = static_cast<uint8_t>(i * 29);
    io::VideoFrameInfo frame;
    frame.frame_id = 0x12345678;
    frame.timestamp_us = 0x0102030405060708ULL;
    frame.keyframe = true;
    auto fragments = io::fragment_video(source, frame);
    require(fragments.front().size() == 300, "Official data segment must be exactly 300 bytes");
    require(fragments.front()[4] == 0x78 && fragments.front()[8] == 8, "Little endian wire layout");
    std::reverse(fragments.begin(), fragments.end());  // Simulated out-of-order delivery.
    std::vector<uint8_t> reconstructed(size);
    for (const auto & data : fragments) {
      io::VideoFragmentInfo parsed;
      require(io::parse_video_fragment(data, parsed), "Valid packet rejected");
      require(parsed.frame.frame_id == frame.frame_id, "Frame ID did not survive serialization");
      require(parsed.frame.timestamp_us == frame.timestamp_us, "Timestamp serialization");
      require(parsed.frame.keyframe && parsed.frame.encoded_size == size, "Frame metadata");
      std::copy_n(
        data.data() + io::kVideoHeaderSize, parsed.payload_size,
        reconstructed.data() + parsed.index * io::kVideoPayloadSize);
      for (size_t i = io::kVideoHeaderSize + parsed.payload_size; i < data.size(); ++i) {
        require(data[i] == 0, "Last fragment padding must be zero");
      }
    }
    require(source == reconstructed, "Out-of-order reassembly changed encoded bytes");
    io::VideoFragmentInfo parsed;
    auto corrupt = fragments.front();
    corrupt[2] = 99;
    require(!io::parse_video_fragment(corrupt, parsed), "Unknown protocol version accepted");
    corrupt = fragments.front();
    corrupt[22] = 0;
    corrupt[23] = 0;
    require(!io::parse_video_fragment(corrupt, parsed), "Zero fragment count accepted");
    corrupt = fragments.front();
    corrupt[24] = 255;
    corrupt[25] = 255;
    require(!io::parse_video_fragment(corrupt, parsed), "Invalid payload size accepted");
  }
  io::VideoStreamConfig invalid;
  invalid.packet_hz = 51;
  bool rejected = false;
  try { invalid.validate(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Official frequency limit not enforced");
}

struct Receiver
{
  int fd;
  uint16_t port;

  explicit Receiver(uint16_t requested = 0)
  {
    fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    require(fd >= 0, "Cannot open local UDP test listener");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(requested);
    if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      close(fd);
      throw std::runtime_error("UDP test port is occupied: " + std::to_string(requested));
    }
    socklen_t length = sizeof(address);
    getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
    port = ntohs(address.sin_port);
  }
  ~Receiver() { close(fd); }
};

void test_transport()
{
  Receiver video;
  io::VideoSender sender(video.port);
  io::VideoFragment fragment{};
  fragment.fill(0xAB);
  require(sender.try_send(fragment), "Nonblocking video send failed");
  std::array<uint8_t, 1024> wire{};
  ssize_t length = -1;
  eventually([&] { length = recv(video.fd, wire.data(), wire.size(), 0); return length >= 0; },
    "Video UDP receive timed out");
  require(length == 306, "Unexpected bridge datagram size");
  require(wire[0] == 0x6B && wire[1] == 1 && wire[2] == 0x10 && wire[3] == 3 &&
    wire[4] == 0x2C && wire[5] == 1, "Bridge message type/command/length changed");
  require(std::equal(fragment.begin(), fragment.end(), wire.begin() + 6), "Bridge payload changed");

  // Bind the original control destination before sending; no real bridge receives this test.
  Receiver control(11451);
  init_send("127.0.0.1");
  send_control(0.25, -0.125, true);
  eventually([&] { length = recv(control.fd, wire.data(), wire.size(), 0); return length >= 0; },
    "Original control UDP receive timed out");
  require(length == 14 && wire[0] == 0x6A, "Original control wire layout changed");
  float yaw = 0, pitch = 0;
  std::memcpy(&yaw, wire.data() + 1, 4);
  std::memcpy(&pitch, wire.data() + 5, 4);
  require(yaw == -0.25F && pitch == -0.125F && wire[9] == 1 && wire[10] == 2,
    "Original yaw sign, pitch, fire or mode changed");
  require(!control_send_active(), "Control activity flag not released");
}

cv::Mat annotated_image()
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(35, 35, 35));
  cv::rectangle(image, {5, 5, 60, 60}, {255, 0, 0}, -1);
  cv::rectangle(image, {575, 5, 60, 60}, {0, 255, 0}, -1);
  cv::rectangle(image, {5, 415, 60, 60}, {255, 255, 255}, -1);
  cv::rectangle(image, {575, 415, 60, 60}, {0, 255, 255}, -1);
  cv::rectangle(image, {120, 100, 400, 280}, {0, 0, 255}, 12);
  cv::putText(image, "annotated full frame", {100, 65}, cv::FONT_HERSHEY_SIMPLEX,
    0.8, {220, 220, 220}, 2);
  return image;
}

#ifdef SP_VISION_ENABLE_VIDEO
// Use a real HEVC decoder, rather than only checking that packet bytes exist.
class Decoder
{
public:
  Decoder()
  {
    const auto * codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    context_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    require(codec && context_ && frame_ && packet_, "HEVC decoder allocation");
    context_->thread_count = 1;
    require(avcodec_open2(context_, codec, nullptr) >= 0, "HEVC decoder initialization");
  }
  ~Decoder()
  {
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    avcodec_free_context(&context_);
  }
  std::vector<cv::Mat> feed(const std::vector<uint8_t> & data)
  {
    if (!data.empty()) {
      require(av_new_packet(packet_, static_cast<int>(data.size())) >= 0, "Decoder packet allocation");
      std::copy(data.begin(), data.end(), packet_->data);
      require(avcodec_send_packet(context_, packet_) >= 0, "HEVC send packet");
      av_packet_unref(packet_);
    } else {
      require(avcodec_send_packet(context_, nullptr) >= 0, "HEVC decoder flush");
    }
    std::vector<cv::Mat> result;
    while (true) {
      const int status = avcodec_receive_frame(context_, frame_);
      if (status == AVERROR(EAGAIN) || status == AVERROR_EOF) break;
      require(status >= 0, "HEVC decode failed");
      cv::Mat bgr(frame_->height, frame_->width, CV_8UC3);
      auto * converter = sws_getContext(
        frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
        frame_->width, frame_->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
      require(converter != nullptr, "Decoder color converter");
      uint8_t * output[] = {bgr.data};
      const int stride[] = {static_cast<int>(bgr.step)};
      sws_scale(converter, frame_->data, frame_->linesize, 0, frame_->height, output, stride);
      sws_freeContext(converter);
      result.push_back(bgr);
    }
    return result;
  }
private:
  AVCodecContext * context_ = nullptr;
  AVFrame * frame_ = nullptr;
  AVPacket * packet_ = nullptr;
};

void check_picture(const cv::Mat & image)
{
  require(image.cols == 256 && image.rows == 192, "Decoded video resolution");
  int red = 0;
  for (int y = 35; y < 157; ++y) {
    for (int x = 42; x < 215; ++x) {
      const auto pixel = image.at<cv::Vec3b>(y, x);
      if (pixel[2] > 120 && pixel[2] > pixel[1] + 60 && pixel[2] > pixel[0] + 60) ++red;
    }
  }
  require(red > 800, "Red aim box missing after encoding/reassembly/decoding");
  const auto blue = image.at<cv::Vec3b>(12, 12);
  const auto green = image.at<cv::Vec3b>(12, 242);
  const auto white = image.at<cv::Vec3b>(180, 12);
  const auto yellow = image.at<cv::Vec3b>(180, 242);
  require(blue[0] > 150 && blue[2] < 90 && green[1] > 150 && green[2] < 90,
    "Upper full-frame corners were cropped/lost");
  require(white[0] > 160 && white[1] > 160 && white[2] > 160 &&
    yellow[1] > 150 && yellow[2] > 150 && yellow[0] < 90,
    "Lower full-frame corners were cropped/lost");
}

void test_codec(const std::filesystem::path & output)
{
  auto source = annotated_image();
  cv::imwrite((output / "annotated_input.png").string(), source);
  cv::Mat resized;
  cv::resize(source, resized, {256, 192}, 0, 0, cv::INTER_AREA);
  io::H265Encoder encoder(256, 192, 15, 48000, 15);
  Decoder decoder;
  std::ofstream stream(output / "local_encoded.h265", std::ios::binary);
  int decoded_count = 0;
  for (int i = 0; i < 12; ++i) {
    const auto encoded = encoder.encode(resized, i == 0 || i == 6);
    require(!encoded.bytes.empty(), "Empty encoded access unit");
    if (i == 0 || i == 6) require(encoded.keyframe, "Forced recovery IDR not produced");
    stream.write(reinterpret_cast<const char *>(encoded.bytes.data()), encoded.bytes.size());
    if (i == 6) {
      Decoder restarted;
      auto pictures = restarted.feed(encoded.bytes);
      auto flushed = restarted.feed({});
      pictures.insert(pictures.end(), flushed.begin(), flushed.end());
      require(pictures.size() == 1, "Recovery keyframe cannot decode without earlier frames");
      check_picture(pictures.front());
    }
    for (const auto & image : decoder.feed(encoded.bytes)) {
      check_picture(image);
      if (decoded_count++ == 0) cv::imwrite((output / "decoded_frame.png").string(), image);
    }
  }
  for (const auto & image : decoder.feed({})) {
    check_picture(image);
    ++decoded_count;
  }
  require(decoded_count == 12, "Encoder or decoder buffered/lost frames");
}

struct CapturedFragment
{
  io::VideoFragment data;
  Clock::time_point sent;
};

void test_streamer(const std::filesystem::path & output)
{
  io::VideoStreamConfig config;
  config.enabled = true;
  config.stats_interval_s = 0.5;
  std::mutex mutex;
  std::vector<CapturedFragment> captured;
  io::VideoStreamer streamer(config, [&](const io::VideoFragment & data) {
    std::lock_guard<std::mutex> lock(mutex);
    captured.push_back({data, Clock::now()});
    return true;
  });
  auto source = annotated_image();
  const auto original = source.clone();
  // Match the main program: retain the annotated full frame across the in-place display resize.
  cv::Mat video_frame = source;
  cv::resize(source, source, {}, 0.5, 0.5);
  const auto start = Clock::now();
  require(streamer.submit(video_frame), "First snapshot rejected");
  video_frame.setTo(cv::Scalar(0, 0, 0));  // Verify snapshot owns annotated pixels.
  source.setTo(cv::Scalar(0, 0, 0));
  while (Clock::now() - start < 2200ms) {
    streamer.submit(original);
    std::this_thread::sleep_for(3ms);
  }
  eventually([&] { return streamer.stats().sent_frames >= 10; }, "Insufficient complete video frames");
  streamer.set_enabled(false);
  std::this_thread::sleep_for(80ms);
  const auto count_when_disabled = streamer.stats().sent_fragments;
  require(!streamer.submit(original), "Disabled streamer accepted frame");
  std::this_thread::sleep_for(80ms);
  require(streamer.stats().sent_fragments == count_when_disabled, "Disabled streamer still transmitting");
  auto stats = streamer.stats();
  require(stats.errors == 0 && stats.encoded >= 10 && stats.last_frame_bytes > 0,
    "Streaming encode statistics invalid");
  require(stats.send_fps > 0 && stats.send_fps <= 16 && stats.bandwidth_bytes_s > 0,
    "Streaming FPS/bandwidth statistics invalid");
  require(stats.submitted <= 35, "Submission exceeded target 15 FPS");

  std::vector<CapturedFragment> packets;
  { std::lock_guard<std::mutex> lock(mutex); packets = captured; }
  std::map<uint32_t, std::vector<io::VideoFragment>> frames;
  std::ofstream packet_file(output / "fragments.bin", std::ios::binary);
  for (size_t i = 0; i < packets.size(); ++i) {
    if (i != 0) require(packets[i].sent - packets[i - 1].sent >= 19ms, "0x0310 pacing burst");
    io::VideoFragmentInfo info;
    require(io::parse_video_fragment(packets[i].data, info), "Streamer emitted invalid fragment");
    frames[info.frame.frame_id].push_back(packets[i].data);
    packet_file.write(reinterpret_cast<const char *>(packets[i].data.data()), io::kVideoDataSize);
  }
  Decoder decoder;
  int pictures = 0;
  std::ofstream stream(output / "reassembled.h265", std::ios::binary);
  for (const auto & entry : frames) {
    io::VideoFragmentInfo first;
    require(io::parse_video_fragment(entry.second.front(), first), "First fragment invalid");
    if (entry.second.size() != first.count) continue;  // Last frame may have been disabled mid-send.
    std::vector<uint8_t> encoded(first.frame.encoded_size);
    for (const auto & fragment : entry.second) {
      io::VideoFragmentInfo info;
      require(io::parse_video_fragment(fragment, info), "Reassembly parse failed");
      std::copy_n(fragment.data() + io::kVideoHeaderSize, info.payload_size,
        encoded.data() + info.index * io::kVideoPayloadSize);
    }
    stream.write(reinterpret_cast<const char *>(encoded.data()), encoded.size());
    for (const auto & image : decoder.feed(encoded)) {
      check_picture(image);
      if (pictures++ == 0) cv::imwrite((output / "streamed_frame.png").string(), image);
    }
  }
  for (const auto & image : decoder.feed({})) { check_picture(image); ++pictures; }
  require(pictures >= 10, "Reassembled HEVC frames cannot decode completely");

  streamer.set_enabled(true);
  eventually([&] { return streamer.submit(original, Clock::now() - 3s); }, "Stale snapshot submission");
  const auto before = stats.queue_drops;
  eventually([&] { return streamer.stats().queue_drops > before; }, "Expired frame was not dropped");
}

void test_backpressure()
{
  io::VideoStreamConfig config;
  config.enabled = true;
  config.max_frame_age_ms = 500;
  config.stats_interval_s = 0.2;
  io::VideoStreamer streamer(config, [](const io::VideoFragment &) { return false; });
  const auto image = annotated_image();
  auto start = Clock::now();
  double max_submit_ms = 0;
  while (Clock::now() - start < 1100ms) {
    const auto before = Clock::now();
    streamer.submit(image);
    max_submit_ms = std::max(max_submit_ms,
      std::chrono::duration<double, std::milli>(Clock::now() - before).count());
    std::this_thread::sleep_for(2ms);
  }
  const auto stats = streamer.stats();
  require(stats.sent_frames == 0 && stats.transport_drops > 0 && stats.queue_drops > 0,
    "Backpressure must drop frames without queue accumulation");
  require(max_submit_ms < 100, "Submit waited for transport backpressure");
  std::cout << "Backpressure max submit: " << max_submit_ms << " ms, queue drops: "
    << stats.queue_drops << ", transport drops: " << stats.transport_drops << '\n';
}

void test_stream_recovery()
{
  io::VideoStreamConfig config;
  config.enabled = true;
  std::mutex mutex;
  std::vector<io::VideoFragment> recovered;
  io::VideoStreamer streamer(config, [&](const io::VideoFragment & fragment) {
    io::VideoFragmentInfo info;
    require(io::parse_video_fragment(fragment, info), "Recovery fragment invalid");
    if (info.frame.frame_id == 0) return false;  // Drop the first encoded frame completely.
    std::lock_guard<std::mutex> lock(mutex);
    recovered.push_back(fragment);
    return true;
  });
  const auto image = annotated_image();
  const auto start = Clock::now();
  while (Clock::now() - start < 1200ms) {
    streamer.submit(image);
    std::this_thread::sleep_for(3ms);
  }
  streamer.set_enabled(false);
  require(streamer.stats().transport_drops > 0, "Recovery test did not drop initial frame");
  std::vector<io::VideoFragment> snapshot;
  { std::lock_guard<std::mutex> lock(mutex); snapshot = recovered; }
  require(!snapshot.empty(), "Stream did not recover after backpressure");
  io::VideoFragmentInfo first;
  require(io::parse_video_fragment(snapshot.front(), first), "Recovery first fragment invalid");
  require(first.frame.keyframe && first.index == 0, "First recovered frame must be IDR");
  std::vector<uint8_t> bytes(first.frame.encoded_size);
  size_t count = 0;
  for (const auto & fragment : snapshot) {
    io::VideoFragmentInfo info;
    require(io::parse_video_fragment(fragment, info), "Recovery packet parse");
    if (info.frame.frame_id != first.frame.frame_id) continue;
    std::copy_n(fragment.data() + io::kVideoHeaderSize, info.payload_size,
      bytes.data() + info.index * io::kVideoPayloadSize);
    ++count;
  }
  require(count == first.count, "Recovered frame incomplete");
  Decoder decoder;
  auto pictures = decoder.feed(bytes);
  auto flushed = decoder.feed({});
  pictures.insert(pictures.end(), flushed.begin(), flushed.end());
  require(pictures.size() == 1, "Recovered stream requires missing reference frames");
  check_picture(pictures.front());
}

void test_hot_reload(const std::filesystem::path & output)
{
  const auto path = output / "hot_reload.yaml";
  { std::ofstream config(path); config << "video:\n  enabled: false\n"; }
  io::VideoStreamer streamer(path.string(), [](const io::VideoFragment &) { return true; });
  require(!streamer.enabled(), "Disabled YAML ignored");
  { std::ofstream config(path); config << "video:\n  enabled: true\n"; }
  eventually([&] { return streamer.enabled(); }, "YAML runtime enable not detected");
  require(streamer.submit(annotated_image()), "Runtime enabled snapshot rejected");
  eventually([&] { return streamer.stats().sent_frames > 0; }, "No frame after runtime enable");
  { std::ofstream config(path); config << "video:\n  enabled: false\n"; }
  eventually([&] { return !streamer.enabled(); }, "YAML runtime disable not detected");
}
#endif
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const std::filesystem::path output = argc > 1 ? argv[1] : "video_test_output";
    std::filesystem::create_directories(output);
    std::filesystem::create_directories("logs");
    test_packets();
    test_transport();
#ifdef SP_VISION_ENABLE_VIDEO
    test_codec(output);
    test_streamer(output);
    test_backpressure();
    test_stream_recovery();
    test_hot_reload(output);
#else
    io::VideoStreamConfig config;
    config.enabled = true;
    io::VideoStreamer streamer(config, [](const io::VideoFragment &) { return true; });
    require(!streamer.enabled() && !streamer.submit(annotated_image()), "Build-time video off ignored");
#endif
    std::cout << "Video tests passed; artifacts: " << std::filesystem::absolute(output) << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Video test failed: " << error.what() << '\n';
    return 1;
  }
}
