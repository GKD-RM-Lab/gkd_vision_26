#include "send_control.hpp"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <cerrno>
#include <cstring>
#include <atomic>
#include "tools/logger.hpp"

// #include "data_manager/parameter_loader.h"  // 该文件不存在，注释掉

int64_t port_num = 11452;
int sockfd;
sockaddr_in serv_addr;
sockaddr_in addr;

namespace
{
std::atomic<unsigned> active_control_sends{0};
}

bool control_send_active() noexcept
{
    return active_control_sends.load(std::memory_order_acquire) != 0;
}

constexpr double RAD2DEG = 57.29577951308232;

enum ROBOT_MODE
{
    ROBOT_NO_FORCE,
    ROBOT_FINISH_INIT,
    ROBOT_FOLLOW_GIMBAL,
    ROBOT_SEARCH,
    ROBOT_IDLE,
    ROBOT_NOT_FOLLOW
};

struct Vison_control
{
    uint8_t header;
    float yaw_set;
    float pitch_set;
    bool fire;

    ROBOT_MODE mode;
} __attribute__((packed));

void init_send(std::string ip)
{
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    addr.sin_port = htons(11451);


    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        printf("can't open socket\n");
    }
    // Video uses a separate socket at priority 0 and cannot hold this send queue.
    const int control_priority = 6;
    setsockopt(sockfd, SOL_SOCKET, SO_PRIORITY, &control_priority, sizeof(control_priority));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port_num);


    if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("can't bind socket fd with port number");
    }
}

void send_control(double yaw_set, double pitch_set, bool fire)
{
    Vison_control pkg{};

    pkg.header = 0x6A;
    pkg.yaw_set = -yaw_set;
    pkg.pitch_set = pitch_set;
    pkg.fire = fire;
    pkg.mode = ROBOT_FOLLOW_GIMBAL;

    //printf("send control\n");
    active_control_sends.fetch_add(1, std::memory_order_acq_rel);
    auto n = sendto(
    sockfd,
    (const char *)(&pkg),
    sizeof(pkg),
    MSG_CONFIRM,
    (const struct sockaddr *)&addr,
    sizeof(addr));
    const int send_error = errno;
    active_control_sends.fetch_sub(1, std::memory_order_release);

    if (n < 0) {
        tools::logger()->warn("[GKDControl] Failed to send control packet: {}", std::strerror(send_error));
    } else {
        const float yaw_rad = pkg.yaw_set;    // Copy packed fields to avoid reference binding issues
        const float pitch_rad = pkg.pitch_set;
        tools::logger()->info(
            "[GKDControl] Send yaw {:.4f} rad ({:.2f} deg), pitch {:.4f} rad ({:.2f} deg), fire {}",
            yaw_rad, yaw_rad * RAD2DEG, pitch_rad, pitch_rad * RAD2DEG, pkg.fire ? "true" : "false");
    }
}
