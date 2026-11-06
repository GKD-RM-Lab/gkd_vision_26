#include "video_sender.hpp"

#include <algorithm>
#include <array>
#include <sys/socket.h>
#include <unistd.h>

#include "send_control.hpp"

namespace io
{
VideoSender::VideoSender(uint16_t port)
{
  destination_.sin_family = AF_INET;
  destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  destination_.sin_port = htons(port);
  socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0) return;
  const int priority = 0;
  const int buffer_size = 2048;
  setsockopt(socket_, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));
  setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
}

VideoSender::~VideoSender()
{
  if (socket_ >= 0) close(socket_);
}

bool VideoSender::try_send(const VideoFragment & fragment) const noexcept
{
  if (fragment.size() <= kVideoHeaderSize || fragment.size() > kVideoDataSize ||
      socket_ < 0 || control_send_active()) return false;
  // Local bridge envelope (not the official UART frame): type, version, command, data length.
  std::array<uint8_t, 6 + kVideoDataSize> packet{};
  packet[0] = 0x6B;
  packet[1] = kVideoVersion;
  packet[2] = static_cast<uint8_t>(kVideoCommand);
  packet[3] = static_cast<uint8_t>(kVideoCommand >> 8);
  packet[4] = static_cast<uint8_t>(fragment.size());
  packet[5] = static_cast<uint8_t>(fragment.size() >> 8);
  std::copy(fragment.begin(), fragment.end(), packet.begin() + 6);
  const auto written = sendto(
    socket_, packet.data(), 6 + fragment.size(), MSG_DONTWAIT,
    reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
  return written == static_cast<ssize_t>(6 + fragment.size());
}
}  // namespace io
