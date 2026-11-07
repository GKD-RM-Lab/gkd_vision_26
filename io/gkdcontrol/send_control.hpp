#pragma once
#include <string>
void init_send(std::string ip);
void send_control(double yaw_set, double pitch_set, bool fire);
// Advisory yield for best-effort video. Control never waits for video.
bool control_send_active() noexcept;
