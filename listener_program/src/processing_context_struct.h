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
    float peakAmplitude = 0.f;              // Signal peak amplitude. Multiple detectors can calculate this

    // Direction estimation metrics
    Eigen::Vector3f directionOfArrival = Eigen::Vector3f::Zero();
    Eigen::VectorXf tdoaVector;
    Eigen::VectorXf crossCorrelationAmps;

    // Time- and frequency-domain metrics
    std::optional<float> oscillationCount;      // from RuCCUSDetector
    std::optional<float> log10SpectrumRatio;    // from RuCCUSFDetector
    std::optional<float> signalDuration;        // from SignalDurationDetector
    std::optional<float> peakFrequency;         // from FPeakLocationDetector
    std::optional<float> centerFrequency;       // from FPeakLocationDetector
    std::optional<float> bw3dB;                 // from BandwidthDetector
    std::optional<float> bw10dB;                // from BandwidthDetector


    // Match flag vectors for classification-style detectors
    std::optional<std::vector<bool>> signalDurationMatches; // from SignalDurationDetector
    std::optional<std::vector<bool>> peakFreqMatches;       // from FPeakLocationDetector (peak freq)
    std::optional<std::vector<bool>> centerFreqMatches;     // from FPeakLocationDetector (center freq)
    std::optional<std::vector<bool>> bw3dBMatches;          // from BandwidthDetector
    std::optional<std::vector<bool>> bw10dBMatches;         // from BandwidthDetector
    std::optional<std::vector<bool>> agreedMatches;         // from AssignmentStage: final assignment

    // Classification results (can be from ONNX or non-NN classification schemes)
    std::optional<std::string> classLabel;  // final reconciled label (or ONNX's direct label)

    // Classification probabilities (come from ONNX only)
    std::optional<float> classProbability;
    std::optional<std::vector<float>> allClassProbabilities;  // Full probability vector, N-class

    int trackingLabel = -1; // from TrackingStage
    bool isValid = false;
};

struct ProcessingContext
{
    const std::shared_ptr<const IFirmware> firmware;

    // Raw data
    std::vector<std::vector<uint8_t>> dataBytes;
    Eigen::MatrixXf channelData;
    std::vector<TimePoint> dataTimes;

    // Extracted snippet for ONNX classification input
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
