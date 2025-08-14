#pragma once

#include "../pch.h"
#include "../utils.h"

class IOutputHandler
{
   public:
    virtual ~IOutputHandler() = default;
    virtual void handleOutput(const DetectionResult& result) = 0;
    virtual void initialize(const TimePoint& timestamp, int numChannels) = 0;
    virtual void finalize() {}
    virtual void flush() {};
};

class FileOutputHandler : public IOutputHandler
{
   public:
    FileOutputHandler(const std::string& loggingDir, bool integrationTesting = false);

    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;

   private:
    void writeBufferToFile();
    void clearBuffer();
    std::vector<std::string> generateChannelComboLabels(const std::string& labelPrefix, int numChannels);

    BufferStruct mBuffer;
    std::string mOutputFile;
    std::string mLoggingDirectory;
    std::chrono::milliseconds mFlushInterval;
    size_t mBufferSizeThreshold;
    std::chrono::time_point<std::chrono::steady_clock> mLastFlushTime;
    bool mIntegrationTesting;
};

class ConsoleOutputHandler : public IOutputHandler
{
   public:
    explicit ConsoleOutputHandler(bool verbose = false);

    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;

   private:
    bool mVerbose;
    size_t mDetectionCount;
};

class NetworkOutputHandler : public IOutputHandler
{
   public:
    NetworkOutputHandler(const std::string& ip, int port);
    ~NetworkOutputHandler() override;

    void handleOutput(const DetectionResult& result) override;
    void initialize(const TimePoint& timestamp, int numChannels) override;
    void finalize() override;

   private:
    std::string mIp;
    int mPort;
    int mSockfd;
    struct sockaddr_in mDest;
    TimePoint mInitTimestamp;
    int mInitNumChannels;
};

class CompositeOutputHandler : public IOutputHandler
{
   public:
    void addHandler(std::unique_ptr<IOutputHandler> handler);
    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;

   private:
    std::vector<std::unique_ptr<IOutputHandler>> mHandlers;
};