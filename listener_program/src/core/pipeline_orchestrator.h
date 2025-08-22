#pragma once

#include "../error_handlers.h"
#include "processing_stages.h"

class IPipelineOrchestrator
{
   public:
    virtual ~IPipelineOrchestrator() = default;
    virtual void addStage(std::unique_ptr<IProcessingStage> stage) = 0;
    virtual bool executeStages(std::shared_ptr<ProcessingContext> context) = 0;
    virtual void initializeStages(std::shared_ptr<ProcessingContext> context) = 0;
    virtual void tickPeriodicStages() = 0;
    virtual size_t getStageCount() const = 0;
    virtual IProcessingStage* getStageIndex(size_t index) const = 0;
    virtual IProcessingStage* getStageName(const std::string& name) const = 0;
};

class PipelineOrchestrator : public IPipelineOrchestrator
{
   public:
    PipelineOrchestrator();

    void addStage(std::unique_ptr<IProcessingStage> stage) override;
    bool executeStages(std::shared_ptr<ProcessingContext> context) override;
    void initializeStages(std::shared_ptr<ProcessingContext> context) override;
    void tickPeriodicStages() override;
    size_t getStageCount() const override;
    IProcessingStage* getStageIndex(size_t index) const override;
    IProcessingStage* getStageName(const std::string& name) const override;

   private:
    std::vector<std::unique_ptr<IProcessingStage>> mStages;

    // Track which stages need periodic ticking using their index in mStages
    std::vector<size_t> mPeriodicStageIndices;
};