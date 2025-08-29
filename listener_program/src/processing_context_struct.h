#pragma once

#include "firmware/firmware_interface.h"
#include "pch.h"
// Forward declarations
/*
class IFirmware;
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;
*/

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

struct DetectionResult
{
    float peakAmplitude = 0.f;
    Eigen::Vector3f directionOfArrival = Eigen::Vector3f::Zero();
    Eigen::VectorXf tdoaVector;
    Eigen::VectorXf crossCorrelationAmps;
    int trackingLabel = -1;
    bool isValid = false;
};

struct ProcessingContext
{
    const std::shared_ptr<const IFirmware> firmware;

    std::vector<std::vector<uint8_t>> dataBytes;
    Eigen::MatrixXf channelData;
    std::vector<TimePoint> dataTimes;

    // frequency domain filtering intermediate data
    Eigen::MatrixXcf frequencyDomainData;
    Eigen::MatrixXcf beforeFilterData;

    DetectionResult currentResult;

    ProcessingContext() = default;

    ProcessingContext(const std::shared_ptr<const IFirmware> firmware)
        : channelData(Eigen::MatrixXf::Zero(firmware->numChannels(), firmware->channelSize())),
          firmware(firmware),
          dataBytes(firmware->numPacketsToDetect())
    {
    }

    void reset() { currentResult = DetectionResult{}; }
};
