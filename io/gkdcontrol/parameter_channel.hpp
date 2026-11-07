#ifndef IO__PARAMETER_CHANNEL_HPP
#define IO__PARAMETER_CHANNEL_HPP

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

#include "tasks/auto_aim/runtime_params.hpp"

namespace io
{
// Loopback-only parameter transport. All JSON and socket I/O stays on the worker.
class ParameterChannel
{
public:
  ParameterChannel(const auto_aim::RuntimeConfig & config, const auto_aim::RuntimeState & initial);
  ~ParameterChannel();
  ParameterChannel(const ParameterChannel &) = delete;
  ParameterChannel & operator=(const ParameterChannel &) = delete;

  bool try_receive(auto_aim::ParameterRequest & request);
  bool try_reply(const auto_aim::ParameterReply & reply);
  void try_publish(const auto_aim::RuntimeState & state);

private:
  void run();
  auto_aim::RuntimeConfig config_;
  int socket_ = -1;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex mutex_;
  std::deque<auto_aim::ParameterRequest> requests_;
  std::deque<auto_aim::ParameterReply> replies_;
  std::optional<auto_aim::ParameterRequest> notice_;
  auto_aim::RuntimeState latest_state_;
};
}  // namespace io
#endif
