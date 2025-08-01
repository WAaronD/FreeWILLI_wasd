#pragma once

#include "firmware/firmware_interface.h"
#include "pch.h"
// Forward declarations
class IFirmware;
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct DetectionResult
{
    TimePoint timestamp;
    float peakAmplitude = 0.0f;
    Eigen::Vector3f directionOfArrival = Eigen::Vector3f::Zero();
    Eigen::VectorXf tdoaVector;
    Eigen::VectorXf crossCorrelationAmps;
    int trackingLabel = -1;
    bool isValid = false;

    DetectionResult() = default;

    DetectionResult(
        const TimePoint& ts, float amp, const Eigen::Vector3f& doa, const Eigen::VectorXf& tdoa,
        const Eigen::VectorXf& xcorr)
        : timestamp(ts),
          peakAmplitude(amp),
          directionOfArrival(doa),
          tdoaVector(tdoa),
          crossCorrelationAmps(xcorr),
          isValid(true)
    {
    }
};

struct ProcessingContext
{
    bool pipelineInitialized;

    Eigen::MatrixXf channelData;
    std::vector<TimePoint> dataTimes;
    std::vector<std::vector<uint8_t>> dataBytes;
    Eigen::MatrixXcf frequencyDomainData;
    Eigen::MatrixXcf beforeFilterData;
    DetectionResult currentResult;
    std::shared_ptr<const IFirmware> firmware;

    // Optional intermediate data
    Eigen::VectorXf tdoaVector;
    Eigen::VectorXf directionOfArrival;
    Eigen::VectorXf crossCorrelationAmps;
    float timeDomainDetectionValue;

    ProcessingContext() : pipelineInitialized(false) {};
    ProcessingContext(std::shared_ptr<const IFirmware> firmware)
        : pipelineInitialized(false),
          channelData(Eigen::MatrixXf::Zero(firmware->numChannels(), firmware->channelSize())),
          firmware(firmware),
          dataBytes(firmware->numPacketsToDetect())
    {
    }

    void reset()
    {
        currentResult = DetectionResult{};
        /*
        tdoaVector.reset();
        directionOfArrival.reset();
        crossCorrelationAmps.reset();
        timeDomainDetectionValue.reset();
        frequencyDomainData.resize(0, 0);
        beforeFilterData.resize(0, 0);
        */
    }
};

struct ProcessingError
{
    std::string stageName;
    std::string errorMessage;
    std::exception_ptr exception;
    ProcessingContext context;

    ProcessingError(const std::string& stage, const std::string& message) : stageName(stage), errorMessage(message) {}

    ProcessingError(
        const std::string& stage, const std::string& message, std::exception_ptr ex, const ProcessingContext& ctx)
        : stageName(stage), errorMessage(message), exception(ex), context(ctx)
    {
    }
};

struct BufferStruct
{
    std::vector<float> mAmps;
    std::vector<float> mDoaX;
    std::vector<float> mDoaY;
    std::vector<float> mDoaZ;
    std::vector<Eigen::VectorXf> mTdoaVector;
    std::vector<Eigen::VectorXf> mXCorrAmps;
    std::vector<TimePoint> mPeakTimes;

    void clear()
    {
        mAmps.clear();
        mDoaX.clear();
        mDoaY.clear();
        mDoaZ.clear();
        mTdoaVector.clear();
        mXCorrAmps.clear();
        mPeakTimes.clear();
    }

    size_t size() const { return mPeakTimes.size(); }

    bool empty() const { return mPeakTimes.empty(); }
};

// ============================================================================
// CORE INTERFACES
// ============================================================================

class IProcessingStage
{
   public:
    virtual ~IProcessingStage() = default;
    virtual bool process(std::shared_ptr<ProcessingContext> context) = 0;
    virtual std::string getName() const = 0;
    virtual void initialize(std::shared_ptr<ProcessingContext> context) {}
};

class IOutputHandler
{
   public:
    virtual ~IOutputHandler() = default;
    virtual void handleOutput(const DetectionResult& result) = 0;
    virtual void flush() = 0;
    virtual void initialize(const TimePoint& timestamp, int numChannels) = 0;
    virtual void finalize() {}
};

class IErrorHandler
{
   public:
    virtual ~IErrorHandler() = default;
    virtual void handleError(const ProcessingError& error) = 0;
};

class IPipelineOrchestrator
{
   public:
    virtual ~IPipelineOrchestrator() = default;
    virtual void addStage(std::unique_ptr<IProcessingStage> stage) = 0;
    virtual bool executeStages(std::shared_ptr<ProcessingContext> context) = 0;
    virtual void initializeStages(std::shared_ptr<ProcessingContext> context) = 0;
};