#pragma once
#include "../pch.h"

class ISocketManager
{
   public:
    virtual ~ISocketManager() = default;

    virtual void restartListener() = 0;

    virtual std::vector<uint8_t>& receiveData(int flags, struct sockaddr* addr, socklen_t* addrlen) = 0;
};