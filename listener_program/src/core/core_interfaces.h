#pragma once

#include "../firmware/firmware_interface.h"
#include "../pch.h"
#include "../structs.h"

// Forward declarations
class IFirmware;
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;

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

class IPipelineOrchestrator
{
   public:
    virtual ~IPipelineOrchestrator() = default;
    virtual void addStage(std::unique_ptr<IProcessingStage> stage) = 0;
    virtual bool executeStages(std::shared_ptr<ProcessingContext> context) = 0;
    virtual void initializeStages(std::shared_ptr<ProcessingContext> context) = 0;
    virtual void tickPeriodicStages() = 0;
};