#include "../../src/io/udp_socket_manager.h"

#include "../../src/io/isocket_manager.h"
#include "../../src/utils.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

TEST(UdpSocketManagerTest, ConstructorCreatesSocketSuccessfully)
{
    SocketVariables socketVars;
    socketVars.port = 8080;
    socketVars.ipAddress = "127.0.0.1";

    EXPECT_NO_THROW(UdpSocketManager socketManager(socketVars));
}

/*
TEST(SocketManagerTest, ConstructorThrowsOnSocketFailure)
{
    SocketVariables socketVars;
    socketVars.udpPort = 8080;
    socketVars.udpIp = "127.0.0.1";

    // Inject failure by setting an invalid socket manually
    int originalSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
    ::close(originalSocket);  // Close the valid socket
    EXPECT_THROW(SocketManager socketManager(socketVars), std::runtime_error);
}
*/

TEST(UdpSocketManagerTest, RestartListenerRecreatesSocketSuccessfully)
{
    SocketVariables socketVars;
    socketVars.port = 8080;
    socketVars.ipAddress = "127.0.0.1";

    UdpSocketManager socketManager(socketVars);

    EXPECT_NO_THROW(socketManager.restartListener());
}

TEST(UdpSocketManagerTest, RestartListenerThrowsIfCloseFails)
{
    SocketVariables socketVars;
    socketVars.port = 8080;
    socketVars.ipAddress = "127.0.0.1";

    UdpSocketManager socketManager(socketVars);

    // Close the socket manually to force `close()` failure
    ::close(socketManager.getSocket());

    EXPECT_THROW(socketManager.restartListener(), std::runtime_error);
}

TEST(UdpSocketManagerTest, ReceiveDataReturnsNegativeOnFailure)
{
    SocketVariables socketVars;
    socketVars.port = 8080;
    socketVars.ipAddress = "127.0.0.1";

    UdpSocketManager socketManager(socketVars);

    struct sockaddr_in addr;
    socklen_t addrLen = sizeof(addr);

    // Ensure the socket is invalid to force recvfrom() failure
    ::close(socketManager.getSocket());

    int bytesReceived = socketManager.receiveData(0, (struct sockaddr*)&addr, &addrLen);
    EXPECT_EQ(bytesReceived, -1);
}

TEST(UdpSocketManagerTest, SetReceiveBufferSize)
{
    SocketVariables socketVars;
    socketVars.port = 8080;
    socketVars.ipAddress = "127.0.0.1";

    UdpSocketManager socketManager(socketVars);

    socketManager.setReceiveBufferSize(4096);
    EXPECT_EQ(socketManager.getReceivedData().size(), 4096);
}
