#include "../processing_context_struct.h"
#include "output_handlers.h"

namespace {
    std::string optionalToString(const std::optional<float>& val)
    {
        return val.has_value() ? std::to_string(*val) : "NaN";
    }
    std::string optionalToString(const std::optional<std::string>& val)  // New!
    {
        return val.has_value() ? *val : "";
    }
}

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

    std::ofstream file(mOutputFile, std::ofstream::out | std::ofstream::trunc);
    if (!file.is_open())
    {
        throw std::runtime_error("Error: Unable to open file for writing: " + mOutputFile);
    }

    // Create column headers
    // std::vector<std::string> columnNames = {"PeakTime", "Amplitude", "DOA_x", "DOA_y", "DOA_z"}; // Old!
    // std::vector<std::string> columnNames = {"PeakTime", "Amplitude", "DOA_x", "DOA_y", "DOA_z", "OC", "Log10SR"}; // New!
    std::vector<std::string> columnNames = {
        "PeakTime", "Amplitude", "DOA_x", "DOA_y", "DOA_z", "OC", "Log10SR",
        "SignalDuration", "PeakFreq", "CenterFreq", "ClassLabel", "ClassProb"  // New!
    };

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

void FileOutputHandler::handleOutput(const ProcessingContext& result)
{
    if (!result.currentResult.isValid)
    {
        return;
    }

    // Reconcile detector-based class labels (only relevant if ONNX didn't already set one)
    std::optional<std::string> reconciledLabel = result.currentResult.classLabel;  // ONNX takes priority
    if (!reconciledLabel.has_value())
    {
        const auto& a = result.currentResult.signalDurationClassLabel;
        const auto& b = result.currentResult.peakLocationClassLabel;
        if (a.has_value() && b.has_value())
        {
            if (*a == *b) reconciledLabel = a;   // agree -> log it
            // else: leave empty, uncertain
        }
        else if (a.has_value())
        {
            reconciledLabel = a;  // only one detector active in pipeline
        }
        else if (b.has_value())
        {
            reconciledLabel = b;
        }
    }

    mBuffer.mAmps.push_back(result.currentResult.peakAmplitude);
    mBuffer.mDoaX.push_back(result.currentResult.directionOfArrival.x());
    mBuffer.mDoaY.push_back(result.currentResult.directionOfArrival.y());
    mBuffer.mDoaZ.push_back(result.currentResult.directionOfArrival.z());
    mBuffer.mTdoaVector.push_back(result.currentResult.tdoaVector);
    mBuffer.mXCorrAmps.push_back(result.currentResult.crossCorrelationAmps);
    mBuffer.mOc.push_back(result.currentResult.oscillationCount);
    mBuffer.mLog10Sr.push_back(result.currentResult.log10SpectrumRatio);
    mBuffer.mSignalDuration.push_back(result.currentResult.signalDuration);   // New!
    mBuffer.mPeakFreq.push_back(result.currentResult.peakFrequency);          // New!
    mBuffer.mCenterFreq.push_back(result.currentResult.centerFrequency);      // New!
    mBuffer.mClassLabel.push_back(reconciledLabel);   // changed: use reconciledLabel, not result.currentResult.classLabel
    mBuffer.mClassProb.push_back(result.currentResult.classProbability);
    mBuffer.mPeakTimes.push_back(result.dataTimes[0]);
}

void FileOutputHandler::flush()
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
        writeBufferToFile();
        clearBuffer();
        mLastFlushTime = std::chrono::steady_clock::now();
    }
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
        mBuffer.mAmps.size() != dataSize || mBuffer.mOc.size() != dataSize || mBuffer.mLog10Sr.size() != dataSize ||
        mBuffer.mSignalDuration.size() != dataSize || mBuffer.mPeakFreq.size() != dataSize ||
        mBuffer.mCenterFreq.size() != dataSize || mBuffer.mClassLabel.size() != dataSize ||
        mBuffer.mClassProb.size() != dataSize)  // New!
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

        // Add optional fields
        rowData.push_back(optionalToString(mBuffer.mOc[i]));
        rowData.push_back(optionalToString(mBuffer.mLog10Sr[i]));
        rowData.push_back(optionalToString(mBuffer.mSignalDuration[i]));  // New!
        rowData.push_back(optionalToString(mBuffer.mPeakFreq[i]));        // New!
        rowData.push_back(optionalToString(mBuffer.mCenterFreq[i]));      // New!
        rowData.push_back(optionalToString(mBuffer.mClassLabel[i]));      // New!
        rowData.push_back(optionalToString(mBuffer.mClassProb[i]));       // New!

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