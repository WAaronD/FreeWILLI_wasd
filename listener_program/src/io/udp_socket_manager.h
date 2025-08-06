#pragma once
#include "../pch.h"
#include "isocket_manager.h"

class SocketVariables;

/**
 * @brief Manages UDP socket operations, including creating, restarting, and configuring the socket.
 */
class UdpSocketManager : public ISocketManager
{
   public:
    explicit UdpSocketManager(const SocketVariables& socketVariables);

    void restartListener() override;

    std::vector<uint8_t>& receiveData(int flags, struct sockaddr* addr, socklen_t* addrlen) override;

    // std::vector<uint8_t>& getReceivedData() override { return mDataBytes; }
    //  void setReceiveBufferSize(size_t newSize) override { mDataBytes.resize(2048); }

   private:
    int mDatagramSocket;  ///< UDP socket descriptor.
    int mUdpPort;  ///< Port number for the UDP connection.
    std::string mUdpIp;  ///< IP address of the data logger or simulator.

    // Store dataBytes as a member to allow easier mocking and testing.
    std::vector<uint8_t> mDataBytes;
};