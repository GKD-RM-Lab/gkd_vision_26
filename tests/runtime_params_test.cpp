#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <thread>

#include "io/gkdcontrol/parameter_channel.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/center_aimer.hpp"
#include "tasks/auto_aim/runtime_params.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

namespace
{
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using namespace auto_aim;

void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}

int64_t unix_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

struct Fixture
{
  std::filesystem::path directory;
  Fixture()
  {
    char path[] = "/tmp/gkd-runtime-test-XXXXXX";
    const char * result = mkdtemp(path);
    require(result != nullptr, "temporary directory");
    directory = result;
  }
  ~Fixture() { std::filesystem::remove_all(directory); }
  std::string config(bool allow_color, uint16_t request_port = 11455, uint16_t status_port = 11456,
                     bool enabled = true)
  {
    auto yaml = YAML::LoadFile(RUNTIME_TEST_CONFIG);
    yaml["runtime_params"]["enabled"] = enabled;
    yaml["runtime_params"]["color_override_enabled"] = allow_color;
    yaml["runtime_params"]["request_port"] = request_port;
    yaml["runtime_params"]["status_port"] = status_port;
    yaml["runtime_params"]["queue_capacity"] = 2;
    yaml["runtime_params"]["status_hz"] = 10;
    const auto path = (directory / (std::to_string(request_port) + ".yaml")).string();
    std::ofstream stream(path);
    stream << yaml;
    return path;
  }
};

ParameterRequest ekf_request(const RuntimeParams & params, const std::string & id = "ekf-1")
{
  ParameterRequest request;
  request.request_id = id;
  request.operation = ParameterOperation::set_ekf;
  request.session_id = params.state().session_id;
  request.base_revision = params.state().revision;
  request.expires_at_unix_ms = unix_ms() + 1000;
  request.process_v1 = 240;
  request.process_v2 = 200;
  return request;
}

void test_transactions(Fixture & fixture)
{
  RuntimeParams params(fixture.config(false), red);
  params.begin_frame(red, 10, 12345678);
  auto request = ekf_request(params);
  auto reply = params.apply(request, unix_ms());
  require(reply.result == "applied" && params.state().revision == 1, "EKF transaction applied");
  require(params.state().process_v1 == 240 && params.state().process_v2 == 200, "both values applied");
  auto retry = params.apply(request, unix_ms() + 3000);
  require(retry.duplicate && retry.applied_revision == 1 && params.state().revision == 1,
    "lost ACK retry must not reapply, even after deadline");
  request.process_v1 = 300;
  require(params.apply(request, unix_ms()).reason == "request_id_conflict", "ID payload collision");

  const auto rejected_unchanged = [&](ParameterRequest bad, const std::string & reason) {
    require(params.apply(bad, unix_ms()).reason == reason, reason);
    require(params.state().revision == 1 && params.state().process_v1 == 240 &&
      params.state().process_v2 == 200, "rejection must preserve entire transaction");
  };
  for (double invalid : {0.0, -1.0, 1000001.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
    auto bad = ekf_request(params, "invalid");
    bad.process_v2 = invalid;
    rejected_unchanged(bad, "invalid_v1_v2");
  }
  auto bad = ekf_request(params, "stale-version");
  bad.base_revision = 0;
  rejected_unchanged(bad, "revision_conflict");
  bad = ekf_request(params, "stale-session");
  bad.session_id = "old-session";
  rejected_unchanged(bad, "session_mismatch");
  bad = ekf_request(params, "expired");
  bad.expires_at_unix_ms = unix_ms() - 1;
  rejected_unchanged(bad, "expired_or_invalid_deadline");
  bad = ekf_request(params, "future");
  bad.expires_at_unix_ms = unix_ms() + 20000;
  rejected_unchanged(bad, "expired_or_invalid_deadline");
  bad = ekf_request(params, "queued-too-long");
  bad.received_at -= 600ms;
  rejected_unchanged(bad, "queue_expired");
  bad = ekf_request(params, "color-disabled");
  bad.operation = ParameterOperation::set_enemy_color;
  bad.confirmation_hold_ms = 500;
  rejected_unchanged(bad, "color_override_disabled");
  ParameterRequest get;
  get.request_id = "read";
  require(params.apply(get, unix_ms()).result == "snapshot", "read current state");
  RuntimeParams restarted(fixture.config(false), red);
  require(restarted.state().session_id != params.state().session_id && restarted.state().revision == 0 &&
    restarted.state().process_v1 == 120 && restarted.state().process_v2 == 100,
    "restart creates new session and restores YAML defaults");
  require(restarted.apply(ekf_request(params, "old-process"), unix_ms()).reason == "session_mismatch",
    "old process command cannot apply after restart");
  RuntimeParams disabled(fixture.config(false, 11455, 11456, false), red);
  require(disabled.apply(ekf_request(disabled), unix_ms()).reason == "disabled", "feature off");
}

void test_colors_and_overlay(Fixture & fixture)
{
  RuntimeParams params(fixture.config(true), red);
  auto request = ekf_request(params, "color-1");
  request.operation = ParameterOperation::set_enemy_color;
  request.color_mode = EnemyColorMode::blue;
  require(params.apply(request, unix_ms()).reason == "color_confirmation_required", "missing hold");
  request.confirmation_hold_ms = 499;
  require(params.apply(request, unix_ms()).reason == "color_confirmation_required", "short hold");
  request.confirmation_hold_ms = 500;
  require(params.apply(request, unix_ms()).result == "applied", "confirmed color transaction");
  params.begin_frame(red, 11, 23456789);
  require(params.state().effective_enemy == blue && params.state().automatic_enemy == red,
    "manual override survives automatic updates");
  request = ekf_request(params, "color-auto");
  request.operation = ParameterOperation::set_enemy_color;
  request.color_mode = EnemyColorMode::automatic;
  request.confirmation_hold_ms = 500;
  require(params.apply(request, unix_ms()).result == "applied" && params.state().effective_enemy == red,
    "return to automatic");
  params.begin_frame(blue, 12, 34567890);
  require(params.state().effective_enemy == blue, "automatic follows source");
  cv::Mat image(1080, 1440, CV_8UC3, cv::Scalar(40, 40, 40));
  const auto before = image.clone();
  params.draw_overlay(image);
  require(cv::norm(image, before) > 0 && image.size() == before.size(), "overlay keeps full size");
  cv::Mat small;
  cv::resize(image, small, {256, 192});
  require(!small.empty(), "overlay supports stream resize");
  cv::imwrite("runtime-params-overlay.png", small);
}

void test_compact_atomic(Fixture & fixture)
{
  RuntimeParams blocked(fixture.config(false), red);
  auto request = ekf_request(blocked, "compact-both");
  request.operation = ParameterOperation::apply; request.sequence = 123;
  request.update_ekf = request.update_color = true;
  request.color_mode = EnemyColorMode::blue; request.confirmation_hold_ms = 500;
  require(blocked.apply(request, unix_ms()).reason == "color_override_disabled" &&
    blocked.state().revision == 0 && blocked.state().process_v1 == 120,
    "combined apply cannot partially change EKF while color is locked");
  RuntimeParams params(fixture.config(true), red);
  request.session_id = params.state().session_id;
  request.confirmation_hold_ms = 450;
  require(params.apply(request, unix_ms()).reason == "color_confirmation_required" &&
    params.state().process_v1 == 120 && params.state().effective_enemy == red,
    "compact rounded-down hold rejects atomically");
  request.confirmation_hold_ms = 500;
  require(params.apply(request, unix_ms()).result == "applied" && params.state().revision == 1 &&
    params.state().process_v1 == 240 && params.state().effective_enemy == blue &&
    params.state().last_sequence == 123 && params.state().last_result == "applied",
    "combined apply and persistent HUD result");
  require(params.apply(request, unix_ms()).duplicate && params.state().revision == 1,
    "compact duplicate must not increment revision");
  auto notice = request; notice.operation = ParameterOperation::bridge_notice;
  notice.request_id = "bridge-failure"; notice.sequence = 124; notice.notice_reason = "command_expired_in_bridge";
  require(params.apply(notice, unix_ms()).reason == notice.notice_reason && params.state().revision == 1 &&
    params.state().last_sequence == 124 && params.state().last_result == "rejected",
    "bridge error reaches HUD without changing parameters");
  params.begin_frame(red, 20, 1234);
  require(params.state().last_sequence == 124, "HUD result persists across frames");
  cv::Mat image(768, 1024, CV_8UC3, cv::Scalar(20,20,20)), before = image.clone();
  params.draw_overlay(image);
  require(cv::norm(image(cv::Rect(0,0,1024,230)), before(cv::Rect(0,0,1024,230))) > 0 &&
    cv::norm(image(cv::Rect(0,250,1024,518)), before(cv::Rect(0,250,1024,518))) == 0,
    "receipt is drawn only at top left on full image");
}

Armor make_armor(Solver & solver, Color color)
{
  const auto points = solver.reproject_armor({3.0, 0.2, 0.1}, CV_PI, small, one);
  Armor armor(color == red ? 4 : 3, 1.0f, cv::boundingRect(points), points);
  armor.color = color;
  armor.name = one;
  armor.type = small;
  armor.priority = ArmorPriority::first;
  return armor;
}

void test_real_target_and_tracker()
{
  Target target(5, 0.3, 0.2, 0.1, 120, 100);
  Target baseline = target;
  const auto x = target.ekf_x();
  const auto p = target.ekf().P;
  target.set_process_noise(240, 200);
  require(target.ekf_x().isApprox(x, 0) && target.ekf().P.isApprox(p, 0), "hot update preserves x/P");
  target.predict(0.02);
  baseline.predict(0.02);
  require(std::abs(target.ekf().P(1, 1) - baseline.ekf().P(1, 1) - 0.02 * 0.02 * 120) < 1e-12,
    "new v1 used in next actual Q");
  require(std::abs(target.ekf().P(7, 7) - baseline.ekf().P(7, 7) - 0.02 * 0.02 * 100) < 1e-12,
    "new v2 used in next actual Q");
  bool threw = false;
  try { target.set_process_noise(1, -1); } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "invalid direct setter rejected");

  Solver solver(RUNTIME_TEST_CONFIG);
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  Color enemy = red;
  Tracker hot(RUNTIME_TEST_CONFIG, solver, enemy);
  Tracker cold(RUNTIME_TEST_CONFIG, solver, enemy);
  auto stamp = Clock::now();
  const auto acquire = [&](Tracker & tracker, Clock::time_point t) {
    std::list<Target> targets;
    for (int i = 0; i < 5; ++i) {
      std::list<Armor> armors{make_armor(solver, red), make_armor(solver, blue)};
      targets = tracker.track(armors, t + std::chrono::milliseconds(i));
      require(armors.size() == 1 && armors.front().color == enemy, "filter uses effective enemy");
      require(!targets.empty(), "synthetic target acquisition");
    }
    require(tracker.state() == "tracking", "acquisition must pass the existing detection count");
    return targets;
  };
  acquire(hot, stamp);
  acquire(cold, stamp);
  hot.set_process_noise(240, 200);
  const auto compare_prediction = [&](Clock::time_point t) {
    std::list<Armor> no_observations;
    const auto updated = hot.track(no_observations, t);
    const auto original = cold.track(no_observations, t);
    require(!updated.empty() && !original.empty(), "temporary-loss prediction remains active");
    require(updated.front().ekf().P(1, 1) > original.front().ekf().P(1, 1) &&
      updated.front().ekf().P(7, 7) > original.front().ekf().P(7, 7), "tracker current target updated");
  };
  compare_prediction(stamp + 10ms);
  hot.reset();
  cold.reset();
  stamp += 20ms;
  acquire(hot, stamp);
  acquire(cold, stamp);
  compare_prediction(stamp + 10ms);  // new targets must also inherit the current settings
  enemy = blue;
  hot.reset();
  std::list<Armor> red_only{make_armor(solver, red)};
  require(hot.track(red_only, stamp + 20ms).empty() && hot.state() == "lost",
    "old red target cleared on effective color change");
  acquire(hot, stamp + 30ms);
}

void test_color_change_caches()
{
  Solver solver(RUNTIME_TEST_CONFIG);
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  auto armor = make_armor(solver, red);
  solver.solve(armor);
  Eigen::VectorXd initial_p = Eigen::VectorXd::Ones(11);
  std::list<Target> targets{Target(armor, Clock::now(), 0.2, 4, initial_p, 120, 100)};
  CenterAimer center(RUNTIME_TEST_CONFIG);
  const io::Command valid_pitch{true, false, 0.0, 0.1};
  const io::Command no_pitch{false, false, 0.0, 0.0};
  require(center.aim(targets, Clock::now(), valid_pitch, 0).control, "prime original pitch cache");
  require(center.aim(targets, Clock::now(), no_pitch, 0).control, "original cached-pitch behavior");
  center.reset_tracking();
  require(!center.aim(targets, Clock::now(), no_pitch, 0).control,
    "new enemy cannot reuse previous enemy pitch before a fresh valid command");
}

struct UdpSocket
{
  int fd = -1;
  uint16_t port = 0;
  UdpSocket()
  {
    fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    require(fd >= 0, "test UDP socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    require(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "test UDP bind");
    socklen_t size = sizeof(address);
    require(getsockname(fd, reinterpret_cast<sockaddr *>(&address), &size) == 0, "test UDP getsockname");
    port = ntohs(address.sin_port);
  }
  ~UdpSocket() { if (fd >= 0) close(fd); }
  void send(uint16_t destination_port, const std::string & bytes)
  {
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(destination_port);
    require(sendto(fd, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr *>(&destination),
      sizeof(destination)) == static_cast<ssize_t>(bytes.size()), "test UDP send");
  }
  Json receive_type(const std::string & type, const std::string & id = "")
  {
    const auto deadline = Clock::now() + 2s;
    while (Clock::now() < deadline) {
      pollfd descriptor{fd, POLLIN, 0};
      if (poll(&descriptor, 1, 20) <= 0) continue;
      char buffer[4096];
      const auto size = recv(fd, buffer, sizeof(buffer), 0);
      if (size <= 0) continue;
      const auto json = Json::parse(buffer, buffer + size);
      if (json.value("type", "") == type && (id.empty() || json.value("request_id", "") == id)) return json;
    }
    throw std::runtime_error("UDP response timeout: " + type + " " + id);
  }
};

ParameterRequest take_request(io::ParameterChannel & channel)
{
  const auto deadline = Clock::now() + 1s;
  ParameterRequest request;
  while (Clock::now() < deadline) {
    if (channel.try_receive(request)) return request;
    std::this_thread::sleep_for(1ms);
  }
  throw std::runtime_error("typed request timeout");
}

void queue_reply(io::ParameterChannel & channel, const ParameterReply & reply)
{
  const auto deadline = Clock::now() + 1s;
  while (!channel.try_reply(reply)) {
    require(Clock::now() < deadline, "reply enqueue timeout");
    std::this_thread::sleep_for(1ms);
  }
}

void test_udp(Fixture & fixture)
{
  UdpSocket status;
  UdpSocket client;
  uint16_t request_port;
  {
    UdpSocket reserved;
    request_port = reserved.port;
  }
  RuntimeParams params(fixture.config(true, request_port, status.port), red);
  io::ParameterChannel channel(params.config(), params.state());
  const auto initial = status.receive_type("status");
  require(initial.at("state").at("session_id") == params.state().session_id, "initial status");
  client.send(request_port, R"({"version":1,"op":"get","request_id":"get-1"})");
  auto request = take_request(channel);
  params.begin_frame(red, 42, 987654321);
  queue_reply(channel, params.apply(request, unix_ms()));
  require(status.receive_type("ack", "get-1").at("result") == "snapshot", "get ACK");

  Json message{{"version", 1}, {"op", "set_ekf"}, {"request_id", "udp-ekf"},
    {"session_id", params.state().session_id}, {"base_revision", 0},
    {"expires_at_unix_ms", unix_ms() + 1000}, {"process_v1", 240}, {"process_v2", 200}};
  client.send(request_port, message.dump());
  request = take_request(channel);
  require(params.state().process_v1 == 120, "worker must not mutate live parameters");
  params.begin_frame(red, 43, 987664321);
  const auto result = params.apply(request, unix_ms());
  queue_reply(channel, result);
  channel.try_publish(params.state());
  const auto ack = status.receive_type("ack", "udp-ekf");
  require(ack.at("op") == "set_ekf", "ACK identifies the applied operation for the compact wire codec");
  require(ack.at("result") == "applied" && ack.at("state").at("revision") == 1 &&
    ack.at("state").at("frame_seq") == 43 && ack.at("state").at("capture_timestamp_us") == 987664321,
    "actual frame application ACK");
  client.send(request_port, message.dump());
  queue_reply(channel, params.apply(take_request(channel), unix_ms()));
  require(status.receive_type("ack", "udp-ekf").at("duplicate") == true && params.state().revision == 1,
    "UDP retry deduplication");

  // A generic EKF request must never carry a color side effect.
  message["mode"] = "blue";
  client.send(request_port, message.dump());
  require(status.receive_type("error").at("reason") == "malformed_request", "mixed operations rejected");
  request = {};
  require(!channel.try_receive(request), "mixed request did not enter queue");
  message.erase("mode");
  message["process_v2"] = true;
  client.send(request_port, message.dump());
  std::this_thread::sleep_for(30ms);
  require(!channel.try_receive(request), "boolean parameter rejected");
  message["process_v2"] = 200;

  message = {{"version", 1}, {"op", "set_enemy_color"}, {"request_id", "udp-color"},
    {"session_id", params.state().session_id}, {"base_revision", 1},
    {"expires_at_unix_ms", unix_ms() + 1000}, {"mode", "blue"},
    {"confirmation", {{"method", "four_key_hold"}, {"held_ms", 500}}}};
  client.send(request_port, message.dump());
  queue_reply(channel, params.apply(take_request(channel), unix_ms()));
  const auto color_ack = status.receive_type("ack", "udp-color");
  require(color_ack.at("result") == "applied" &&
    color_ack.at("state").at("effective_enemy_color") == "blue", "independent color operation");

  message["request_id"] = "udp-color-unconfirmed";
  message.erase("confirmation");
  client.send(request_port, message.dump());
  std::this_thread::sleep_for(30ms);
  require(!channel.try_receive(request), "color request without confirmation never enters queue");

  for (int i = 0; i < 20; ++i) {
    client.send(request_port, Json{{"version", 1}, {"op", "get"},
      {"request_id", "flood-" + std::to_string(i)}}.dump());
  }
  std::this_thread::sleep_for(50ms);
  int count = 0;
  while (channel.try_receive(request)) ++count;
  require(count == 2, "receive queue is bounded; overflow drops new commands");
  client.send(request_port, std::string(3000, 'x'));
  std::this_thread::sleep_for(30ms);
  require(!channel.try_receive(request), "oversized datagram rejected");

  // Disabling the optional transport must not bind or create a worker.
  auto disabled_config = params.config();
  disabled_config.enabled = false;
  io::ParameterChannel disabled(disabled_config, params.state());
  require(!disabled.try_receive(request) && !disabled.try_reply(result), "disabled transport no-op");
}
}  // namespace

int main()
{
  try {
    Fixture fixture;
    test_transactions(fixture);
    test_colors_and_overlay(fixture);
    test_compact_atomic(fixture);
    test_real_target_and_tracker();
    test_color_change_caches();
    test_udp(fixture);
    std::cout << "Runtime parameter transactions, color gating, real EKF/tracker and UDP tests passed.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Runtime parameter test failed: " << error.what() << '\n';
    return 1;
  }
}
