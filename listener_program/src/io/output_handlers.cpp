#include "output_handlers.h"

#include <iostream>
#include <stdexcept>

FileOutputHandler::FileOutputHandler(const std::string& loggingDir, bool integrationTesting)
    : mLoggingDirectory(loggingDir),
      mFlushInterval(std::chrono::seconds(30)),
      mBufferSizeThreshold(1000),
      mLastFlushTime(std::chrono::steady_clock::now()),
      mIntegrationTesting(integrationTesting)
{
    // Ensure logging directory ends with separator
    if (!mLoggingDirectory.empty() && mLoggingDirectory.back() != '/' && mLoggingDirectory.back() != '\\')
    {
        mLoggingDirectory += '/';
    }
}

void FileOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    mOutputFile = mLoggingDirectory + convertTimePointToString(timestamp);
    std::cout << "Creating and writing to file: " << mOutputFile << std::endl;

    initializeFileWithHeaders(numChannels);
}

void FileOutputHandler::initializeFileWithHeaders(int numChannels)
{
    std::ofstream file(mOutputFile, std::ofstream::out | std::ofstream::trunc);
    if (!file.is_open())
    {
        throw std::runtime_error("Error: Unable to open file for writing: " + mOutputFile);
    }

    // Create column headers
    std::vector<std::string> columnNames = {"PeakTime", "Amplitude", "DOA_x", "DOA_y", "DOA_z"};

    // Generate TDOA and XCorr labels for channel combinations
    std::vector<std::string> tdoaLabels = generateChannelComboLabels("TDOA", numChannels);
    std::vector<std::string> xcorrLabels = generateChannelComboLabels("XCorr", numChannels);

    // Combine all column names
    columnNames.insert(columnNames.end(), tdoaLabels.begin(), tdoaLabels.end());
    columnNames.insert(columnNames.end(), xcorrLabels.begin(), xcorrLabels.end());

    // Write header row
    for (size_t i = 0; i < columnNames.size(); ++i)
    {
        file << columnNames[i];
        if (i < columnNames.size() - 1)
        {
            file << ",";
        }
    }
    file << std::endl;

    file.close();
}

std::vector<std::string> FileOutputHandler::generateChannelComboLabels(const std::string& labelPrefix, int numChannels)
{
    std::vector<std::string> labels;

    // Generate labels for all unique channel combinations (1-indexed)
    for (int signalChannel = 1; signalChannel < numChannels; ++signalChannel)
    {
        for (int referenceChannel = signalChannel + 1; referenceChannel <= numChannels; ++referenceChannel)
        {
            labels.push_back(labelPrefix + std::to_string(signalChannel * 10 + referenceChannel));
        }
    }

    return labels;
}

void FileOutputHandler::handleOutput(const DetectionResult& result)
{
    if (!result.isValid)
    {
        return;
    }

    appendToBuffer(result);
    flushIfNecessary();
}

void FileOutputHandler::appendToBuffer(const DetectionResult& result)
{
    mBuffer.mAmps.push_back(result.peakAmplitude);
    mBuffer.mDoaX.push_back(result.directionOfArrival.x());
    mBuffer.mDoaY.push_back(result.directionOfArrival.y());
    mBuffer.mDoaZ.push_back(result.directionOfArrival.z());
    mBuffer.mTdoaVector.push_back(result.tdoaVector);
    mBuffer.mXCorrAmps.push_back(result.crossCorrelationAmps);
    mBuffer.mPeakTimes.push_back(result.timestamp);
}

void FileOutputHandler::flushIfNecessary()
{
    if (mBuffer.empty())
    {
        return;
    }

    auto timeSinceLastFlush =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - mLastFlushTime);

    bool shouldFlush =
        mIntegrationTesting || mBuffer.size() >= mBufferSizeThreshold || mFlushInterval <= timeSinceLastFlush;

    if (shouldFlush)
    {
        flush();
    }
}

void FileOutputHandler::flush()
{
    if (mBuffer.empty())
    {
        return;
    }

    writeBufferToFile();
    clearBuffer();
    mLastFlushTime = std::chrono::steady_clock::now();
}

void FileOutputHandler::writeBufferToFile()
{
    std::ofstream file(mOutputFile, std::ofstream::out | std::ofstream::app);
    if (!file.is_open())
    {
        throw std::runtime_error("Error: Unable to open file for appending: " + mOutputFile);
    }

    size_t dataSize = mBuffer.size();

    // Validate buffer consistency
    if (mBuffer.mDoaX.size() != dataSize || mBuffer.mDoaY.size() != dataSize || mBuffer.mDoaZ.size() != dataSize ||
        mBuffer.mTdoaVector.size() != dataSize || mBuffer.mXCorrAmps.size() != dataSize ||
        mBuffer.mAmps.size() != dataSize)
    {
        throw std::runtime_error("Error: Mismatched buffer sizes in BufferStruct.");
    }

    // Determine number of channel pairs from first TDOA vector
    int numChannelPairs = 0;
    if (!mBuffer.mTdoaVector.empty())
    {
        numChannelPairs = mBuffer.mTdoaVector[0].size();
    }

    // Write each detection result as a row
    for (size_t i = 0; i < dataSize; ++i)
    {
        std::vector<std::string> rowData;

        // Convert timestamp to microseconds since epoch
        auto timePoint = mBuffer.mPeakTimes[i];
        auto timeSinceEpoch = std::chrono::duration_cast<std::chrono::microseconds>(timePoint.time_since_epoch());
        rowData.emplace_back(std::to_string(timeSinceEpoch.count()));

        // Add basic detection data
        rowData.push_back(std::to_string(mBuffer.mAmps[i]));
        rowData.push_back(std::to_string(mBuffer.mDoaX[i]));
        rowData.push_back(std::to_string(mBuffer.mDoaY[i]));
        rowData.push_back(std::to_string(mBuffer.mDoaZ[i]));

        // Add TDOA values
        const Eigen::VectorXf& tdoaVec = mBuffer.mTdoaVector[i];
        if (tdoaVec.size() != numChannelPairs)
        {
            throw std::runtime_error("Error: Inconsistent TDOA vector size at index " + std::to_string(i));
        }
        for (int j = 0; j < tdoaVec.size(); ++j)
        {
            rowData.push_back(std::to_string(tdoaVec[j]));
        }

        // Add cross-correlation amplitudes
        const Eigen::VectorXf& xcorrVec = mBuffer.mXCorrAmps[i];
        if (xcorrVec.size() != numChannelPairs)
        {
            throw std::runtime_error("Error: Inconsistent XCorr vector size at index " + std::to_string(i));
        }
        for (int j = 0; j < xcorrVec.size(); ++j)
        {
            rowData.push_back(std::to_string(xcorrVec[j]));
        }

        // Write row to file
        for (size_t k = 0; k < rowData.size(); ++k)
        {
            file << rowData[k];
            if (k < rowData.size() - 1)
            {
                file << ",";
            }
        }
        file << std::endl;
    }

    file.close();
}

void FileOutputHandler::clearBuffer() { mBuffer.clear(); }

void FileOutputHandler::finalize()
{
    if (!mBuffer.empty())
    {
        flush();
    }
}

// ============================================================================
// COMPOSITE OUTPUT HANDLER IMPLEMENTATION
// ============================================================================

void CompositeOutputHandler::addHandler(std::unique_ptr<IOutputHandler> handler)
{
    mHandlers.push_back(std::move(handler));
}

void CompositeOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->initialize(timestamp, numChannels);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error initializing output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::handleOutput(const DetectionResult& result)
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->handleOutput(result);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error in output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::flush()
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->flush();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error flushing output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::finalize()
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->finalize();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error finalizing output handler: " << e.what() << std::endl;
        }
    }
}

// ============================================================================
// CONSOLE OUTPUT HANDLER IMPLEMENTATION
// ============================================================================

ConsoleOutputHandler::ConsoleOutputHandler(bool verbose) : mVerbose(verbose), mDetectionCount(0) {}

void ConsoleOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    std::cout << "=== Console Output Handler Initialized ===" << std::endl;
    std::cout << "Start time: " << convertTimePointToString(timestamp) << std::endl;
    std::cout << "Number of channels: " << numChannels << std::endl;
    std::cout << "Verbose mode: " << (mVerbose ? "ON" : "OFF") << std::endl;
    std::cout << "===========================================" << std::endl;
}

void ConsoleOutputHandler::handleOutput(const DetectionResult& result)
{
    if (!result.isValid)
    {
        return;
    }

    ++mDetectionCount;

    if (mVerbose)
    {
        // Detailed output
        auto timeSinceEpoch =
            std::chrono::duration_cast<std::chrono::microseconds>(result.timestamp.time_since_epoch());

        std::cout << "\n=== Detection #" << mDetectionCount << " ===" << std::endl;
        std::cout << "Timestamp: " << timeSinceEpoch.count() << " μs" << std::endl;
        std::cout << "Peak Amplitude: " << std::fixed << std::setprecision(6) << result.peakAmplitude << std::endl;
        std::cout << "Direction of Arrival: [" << std::fixed << std::setprecision(4) << result.directionOfArrival.x()
                  << ", " << result.directionOfArrival.y() << ", " << result.directionOfArrival.z() << "]" << std::endl;

        if (result.tdoaVector.size() > 0)
        {
            std::cout << "TDOA Vector: [";
            for (int i = 0; i < result.tdoaVector.size(); ++i)
            {
                std::cout << std::fixed << std::setprecision(6) << result.tdoaVector[i];
                if (i < result.tdoaVector.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        if (result.trackingLabel >= 0)
        {
            std::cout << "Tracking Label: " << result.trackingLabel << std::endl;
        }

        std::cout << "==============================" << std::endl;
    }
    else
    {
        // Compact output
        std::cout << "Detection #" << mDetectionCount << " | Amp: " << std::fixed << std::setprecision(3)
                  << result.peakAmplitude << " | DOA: [" << std::fixed << std::setprecision(2)
                  << result.directionOfArrival.x() << "," << result.directionOfArrival.y() << ","
                  << result.directionOfArrival.z() << "]";

        if (result.trackingLabel >= 0)
        {
            std::cout << " | Track: " << result.trackingLabel;
        }

        std::cout << std::endl;
    }
}

void ConsoleOutputHandler::flush() { std::cout.flush(); }

void ConsoleOutputHandler::finalize()
{
    std::cout << "\n=== Console Output Handler Summary ===" << std::endl;
    std::cout << "Total detections processed: " << mDetectionCount << std::endl;
    std::cout << "=======================================" << std::endl;
}

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

void NetworkOutputHandler::handleOutput(const DetectionResult& /*result*/)
{
    const char* msg = "Hello, UDP!";
    std::cout << "sending mesage!!!!! \n";
    ssize_t sent =
        ::sendto(mSockfd, msg, std::strlen(msg), 0, reinterpret_cast<struct sockaddr*>(&mDest), sizeof(mDest));
    if (sent < 0)
    {
        throw std::runtime_error("NetworkOutputHandler: sendto() failed");
    }
}

void NetworkOutputHandler::flush() {}

void NetworkOutputHandler::finalize()
{
    // no-op
}