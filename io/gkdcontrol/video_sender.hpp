#ifndef IO__VIDEO_SENDER_HPP
#define IO__VIDEO_SENDER_HPP

#include <cstdint>
#include <netinet/in.h>

#include "io/video/video_packet.hpp"

namespace io
{
// Separate best-effort UDP socket: no shared mutex or kernel send queue with control.
// The external bridge must arbitrate its physical UART with control-first scheduling.
class VideoSender
{
public:
  explicit VideoSender(uint16_t port);
  ~VideoSender();
  VideoSender(const VideoSender &) = delete;
  VideoSender & operator=(const VideoSender &) = delete;
  bool try_send(const VideoFragment & fragment) const noexcept;

private:
  int socket_ = -1;
  sockaddr_in destination_{};
};
}  // namespace io

#endif  // IO__VIDEO_SENDER_HPP
