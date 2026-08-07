#pragma once

#include "firmware/firmware_interface.h"
#include "pch.h"
#include <limits>
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
    // Eigen::Vector3f directionOfArrival = Eigen::Vector3f::Zero(); // Old!
    Eigen::Vector3f directionOfArrival = Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()); // New!
    Eigen::VectorXf tdoaVector;
    Eigen::VectorXf crossCorrelationAmps;
    std::optional<float> oscillationCount; // New! Optional oscillation count
    std::optional<float> log10SpectrumRatio; // New! Optional log10 spectrum ratio
    std::optional<std::string> classLabel;      // "Ziphius" or "WBAT"
    std::optional<float> classProbability;      // probability of the winning class
    int trackingLabel = -1;
    bool isValid = false;
};

struct ProcessingContext
{
    const std::shared_ptr<const IFirmware> firmware;

    std::vector<std::vector<uint8_t>> dataBytes;
    Eigen::MatrixXf channelData;
    std::vector<TimePoint> dataTimes;

    Eigen::VectorXf classificationSnippet;  // 176-sample peak-centered window for ONNX input

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
