#include "h265_encoder.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace io
{
namespace
{
void check(int result, const char * operation)
{
  if (result >= 0) return;
  char error[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(result, error, sizeof(error));
  throw std::runtime_error(std::string(operation) + ": " + error);
}
}  // namespace

struct H265Encoder::Impl
{
  AVCodecContext * context = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * packet = nullptr;
  SwsContext * scaler = nullptr;
  int64_t pts = 0;

  ~Impl()
  {
    sws_freeContext(scaler);
    av_packet_free(&packet);
    av_frame_free(&frame);
    avcodec_free_context(&context);
  }
};

H265Encoder::H265Encoder(int width, int height, int fps, int bitrate, int gop)
: impl_(std::make_unique<Impl>())
{
  if (width < 16 || height < 16 || width > 4096 || height > 4096 || width % 2 || height % 2 ||
      fps < 1 || fps > 60 || bitrate < 1000 || bitrate > 100000000 || gop < 1 || gop > 600) {
    throw std::invalid_argument("Invalid H.265 encoder configuration");
  }
  const auto * codec = avcodec_find_encoder_by_name("libx265");
  if (!codec) throw std::runtime_error("FFmpeg libx265 encoder is unavailable");
  auto & state = *impl_;
  state.context = avcodec_alloc_context3(codec);
  state.frame = av_frame_alloc();
  state.packet = av_packet_alloc();
  if (!state.context || !state.frame || !state.packet) throw std::bad_alloc();
  auto * ctx = state.context;
  ctx->width = width;
  ctx->height = height;
  ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  ctx->time_base = AVRational{1, fps};
  ctx->framerate = AVRational{fps, 1};
  ctx->bit_rate = bitrate;
  ctx->rc_max_rate = bitrate;
  ctx->rc_buffer_size = bitrate / 2;
  ctx->gop_size = gop;
  ctx->max_b_frames = 0;
  ctx->thread_count = 1;
  ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
  check(av_opt_set(ctx->priv_data, "preset", "ultrafast", 0), "x265 preset");
  check(av_opt_set(ctx->priv_data, "tune", "zerolatency", 0), "x265 tune");
  // No lookahead, B frames, NUMA pools or verbose SEI. Repeat VPS/SPS/PPS on each IDR.
  const std::string params =
    "pools=none:frame-threads=1:wpp=0:bframes=0:rc-lookahead=0:repeat-headers=1:"
    "annexb=1:info=0:open-gop=0:scenecut=0:log-level=error:keyint=" +
    std::to_string(gop) + ":min-keyint=1:vbv-maxrate=" + std::to_string(bitrate / 1000) +
    ":vbv-bufsize=" + std::to_string(std::max(1, bitrate / 2000));
  check(av_opt_set(ctx->priv_data, "x265-params", params.c_str(), 0), "x265 parameters");
  check(avcodec_open2(ctx, codec, nullptr), "avcodec_open2");
  state.frame->format = ctx->pix_fmt;
  state.frame->width = width;
  state.frame->height = height;
  check(av_frame_get_buffer(state.frame, 32), "av_frame_get_buffer");
  state.scaler = sws_getContext(
    width, height, AV_PIX_FMT_BGR24, width, height, AV_PIX_FMT_YUV420P,
    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  if (!state.scaler) throw std::runtime_error("Cannot create BGR to YUV420P converter");
}

H265Encoder::~H265Encoder() = default;

EncodedVideo H265Encoder::encode(const cv::Mat & bgr, bool force_keyframe)
{
  auto & state = *impl_;
  if (bgr.type() != CV_8UC3 || bgr.cols != state.context->width ||
      bgr.rows != state.context->height || bgr.step > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("Encoder expects configured-size CV_8UC3 BGR image");
  }
  check(av_frame_make_writable(state.frame), "av_frame_make_writable");
  const uint8_t * source[] = {bgr.data};
  const int strides[] = {static_cast<int>(bgr.step)};
  if (sws_scale(
        state.scaler, source, strides, 0, bgr.rows, state.frame->data,
        state.frame->linesize) != bgr.rows) {
    throw std::runtime_error("Incomplete BGR to YUV conversion");
  }
  state.frame->pts = state.pts++;
  state.frame->pict_type = force_keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
  check(avcodec_send_frame(state.context, state.frame), "avcodec_send_frame");
  EncodedVideo result;
  while (true) {
    const auto status = avcodec_receive_packet(state.context, state.packet);
    if (status == AVERROR(EAGAIN) || status == AVERROR_EOF) break;
    check(status, "avcodec_receive_packet");
    result.keyframe = result.keyframe || (state.packet->flags & AV_PKT_FLAG_KEY);
    result.bytes.insert(
      result.bytes.end(), state.packet->data, state.packet->data + state.packet->size);
    av_packet_unref(state.packet);
  }
  // zerolatency must deliver the current access unit synchronously.
  if (result.bytes.empty()) throw std::runtime_error("H.265 encoder unexpectedly buffered a frame");
  return result;
}
}  // namespace io
