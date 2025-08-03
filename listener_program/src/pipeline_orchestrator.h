#pragma once

#include <memory>
#include <vector>

#include "interfaces.h"

// ============================================================================
// PIPELINE ORCHESTRATOR IMPLEMENTATION
// ============================================================================

class PipelineOrchestrator : public IPipelineOrchestrator
{
   private:
    std::vector<std::unique_ptr<IProcessingStage>> mStages;

    // Track which stages need periodic ticking using their index in mStages
    std::vector<size_t> mPeriodicStageIndices;

    bool mInitialized;

   public:
    PipelineOrchestrator();

    void addStage(std::unique_ptr<IProcessingStage> stage) override;
    bool executeStages(std::shared_ptr<ProcessingContext> context) override;
    void initializeStages(std::shared_ptr<ProcessingContext> context) override;
    void tickPeriodicStages() override;
    size_t getStageCount() const;
    const IProcessingStage* getStage(size_t index) const;
};