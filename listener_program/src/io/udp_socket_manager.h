#pragma once
#include "../pch.h"
#include "socket_manager_interface.h"

/**
 * @brief Manages UDP socket operations, including creating, restarting, and configuring the socket.
 */
class UdpSocketManager : public ISocketManager
{
   public:
    explicit UdpSocketManager(const std::string& ipAddress, int port);

    void restartListener() override;

    std::vector<uint8_t>& receiveData(int flags, struct sockaddr* addr, socklen_t* addrlen) override;

   private:
    int mDatagramSocket;
    int mUdpPort;
    std::string mUdpIp;

    // Store dataBytes as a member to allow easier mocking and testing.
    std::vector<uint8_t> mDataBytes;
};