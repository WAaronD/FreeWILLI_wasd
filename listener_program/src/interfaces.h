#pragma once

#include "firmware/firmware_interface.h"
#include "pch.h"
#include "structs.h"

// Forward declarations
class IFirmware;
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;

// using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

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
    virtual bool requiresPeriodicTick() const { return false; }
    virtual void tick() {};
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
    virtual void tickPeriodicStages() = 0;
};