#include "../processing_context_struct.h"
#include "output_handlers.h"

NetworkOutputHandler::NetworkOutputHandler(const std::string& ip, int port) : mIp(ip), mPort(port), mSockfd(-1)
{
    // 1) create socket
    mSockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (mSockfd < 0)
    {
        throw std::runtime_error("NetworkOutputHandler: socket() failed");
    }

    // 2) fill out destination address struct
    std::memset(&mDest, 0, sizeof(mDest));
    mDest.sin_family = AF_INET;
    mDest.sin_port = htons(mPort);
    if (::inet_pton(AF_INET, mIp.c_str(), &mDest.sin_addr) != 1)
    {
        ::close(mSockfd);
        throw std::invalid_argument("NetworkOutputHandler: invalid IP address");
    }
}

NetworkOutputHandler::~NetworkOutputHandler()
{
    if (mSockfd >= 0)
    {
        ::close(mSockfd);
    }
}

void NetworkOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    mInitTimestamp = timestamp;
    mInitNumChannels = numChannels;
}

void NetworkOutputHandler::handleOutput(const ProcessingContext& /*result*/)
{
    const char* msg = "Hello, UDP!";
    ssize_t sent =
        ::sendto(mSockfd, msg, std::strlen(msg), 0, reinterpret_cast<struct sockaddr*>(&mDest), sizeof(mDest));
    if (sent < 0)
    {
        throw std::runtime_error("NetworkOutputHandler: sendto() failed");
    }
}

void NetworkOutputHandler::finalize()
{
    // no-op
}