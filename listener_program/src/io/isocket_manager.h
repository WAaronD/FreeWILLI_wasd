#pragma once
#include "../pch.h"

class ISocketManager
{
   public:
    virtual ~ISocketManager() = default;

    virtual void restartListener() = 0;
    virtual int getSocket() const = 0;
    virtual int getPort() const = 0;
    virtual std::string getIp() const = 0;

    virtual int receiveData(int flags, struct sockaddr* addr, socklen_t* addrlen) = 0;

    virtual std::vector<uint8_t>& getReceivedData() = 0;
    virtual void setReceiveBufferSize(size_t newSize) = 0;
};