#include "../processing_context_struct.h"
#include "output_handlers.h"

ConsoleOutputHandler::ConsoleOutputHandler(bool verbose) : mVerbose(verbose), mDetectionCount(0) {}

void ConsoleOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    std::cout << "=== Console Output Handler Initialized ===" << std::endl;
    std::cout << "Start time: " << convertTimePointToString(timestamp) << std::endl;
    std::cout << "Number of channels: " << numChannels << std::endl;
    std::cout << "Verbose mode: " << (mVerbose ? "ON" : "OFF") << std::endl;
    std::cout << "===========================================" << std::endl;
}

void ConsoleOutputHandler::handleOutput(const ProcessingContext& result)
{
    if (!result.currentResult.isValid)
    {
        return;
    }

    ++mDetectionCount;

    if (mVerbose)
    {
        // Detailed output
        auto timeSinceEpoch =
            std::chrono::duration_cast<std::chrono::microseconds>(result.dataTimes[0].time_since_epoch());

        std::cout << "\n=== Detection #" << mDetectionCount << " ===" << std::endl;
        std::cout << "Timestamp: " << timeSinceEpoch.count() << " μs" << std::endl;
        std::cout << "Peak Amplitude: " << std::fixed << std::setprecision(6) << result.currentResult.peakAmplitude
                  << std::endl;
        std::cout << "Direction of Arrival: [" << std::fixed << std::setprecision(4)
                  << result.currentResult.directionOfArrival.x() << ", " << result.currentResult.directionOfArrival.y()
                  << ", " << result.currentResult.directionOfArrival.z() << "]" << std::endl;

        if (result.currentResult.tdoaVector.size() > 0)
        {
            std::cout << "TDOA Vector: [";
            for (int i = 0; i < result.currentResult.tdoaVector.size(); ++i)
            {
                std::cout << std::fixed << std::setprecision(6) << result.currentResult.tdoaVector[i];
                if (i < result.currentResult.tdoaVector.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        if (result.currentResult.trackingLabel >= 0)
        {
            std::cout << "Tracking Label: " << result.currentResult.trackingLabel << std::endl;
        }

        std::cout << "==============================" << std::endl;
    }
    else
    {
        // Compact output
        std::cout << "Detection #" << mDetectionCount << " | Amp: " << std::fixed << std::setprecision(3)
                  << result.currentResult.peakAmplitude << " | DOA: [" << std::fixed << std::setprecision(2)
                  << result.currentResult.directionOfArrival.x() << "," << result.currentResult.directionOfArrival.y()
                  << "," << result.currentResult.directionOfArrival.z() << "]";

        if (result.currentResult.trackingLabel >= 0)
        {
            std::cout << " | Track: " << result.currentResult.trackingLabel;
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