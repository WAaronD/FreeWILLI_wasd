#pragma once

#include <fstream>
#include <iomanip>
#include <sstream>

#include "interfaces.h"
#include "pch.h"
#include "utils.h"

// ============================================================================
// OUTPUT HANDLER IMPLEMENTATIONS
// ============================================================================

class FileOutputHandler : public IOutputHandler
{
   private:
    BufferStruct mBuffer;
    std::string mOutputFile;
    std::string mLoggingDirectory;
    std::chrono::milliseconds mFlushInterval;
    size_t mBufferSizeThreshold;
    std::chrono::time_point<std::chrono::steady_clock> mLastFlushTime;
    bool mIntegrationTesting;
    bool mInitialized;

   public:
    FileOutputHandler(const std::string& loggingDir, bool integrationTesting = false);

    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;

   private:
    void initializeFileWithHeaders(int numChannels);
    void appendToBuffer(const DetectionResult& result);
    void flushIfNecessary();
    void writeBufferToFile();
    void clearBuffer();
    std::vector<std::string> generateChannelComboLabels(const std::string& labelPrefix, int numChannels);
};

class CompositeOutputHandler : public IOutputHandler
{
   private:
    std::vector<std::unique_ptr<IOutputHandler>> mHandlers;

   public:
    void addHandler(std::unique_ptr<IOutputHandler> handler);
    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;
};

class ConsoleOutputHandler : public IOutputHandler
{
   private:
    bool mVerbose;
    size_t mDetectionCount;

   public:
    explicit ConsoleOutputHandler(bool verbose = false);

    void initialize(const TimePoint& timestamp, int numChannels) override;
    void handleOutput(const DetectionResult& result) override;
    void flush() override;
    void finalize() override;
};