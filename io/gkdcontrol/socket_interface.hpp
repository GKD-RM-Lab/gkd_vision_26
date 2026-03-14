#ifndef __SOCKET_INTERFACE__
#define __SOCKET_INTERFACE__

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <string>

struct ReceiveGimbalInfo
{
	uint8_t header;
    float yaw;
    float pitch;
    bool red;
} __attribute__((packed));

namespace IO
{
    class Server_socket_interface
    {
       public:
        Server_socket_interface();
        ~Server_socket_interface();
        void task();
        ReceiveGimbalInfo pkg{};

       private:
        int64_t port_num;
        int sockfd;

        sockaddr_in serv_addr;
        std::map<uint8_t, sockaddr_in> clients;
        std::map<uint8_t, uint8_t> connections;

        char buffer[256];

       private:
    };
}  // namespace IO
#endif
