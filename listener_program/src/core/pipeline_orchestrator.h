#pragma once

#include <memory>
#include <vector>

#include "core_interfaces.h"

// ============================================================================
// PIPELINE ORCHESTRATOR IMPLEMENTATION
// ============================================================================

class PipelineOrchestrator : public IPipelineOrchestrator
{
   public:
    PipelineOrchestrator();

    void addStage(std::unique_ptr<IProcessingStage> stage) override;
    bool executeStages(std::shared_ptr<ProcessingContext> context) override;
    void initializeStages(std::shared_ptr<ProcessingContext> context) override;
    void tickPeriodicStages() override;
    size_t getStageCount() const;
    const IProcessingStage* getStage(size_t index) const;

   private:
    std::vector<std::unique_ptr<IProcessingStage>> mStages;

    // Track which stages need periodic ticking using their index in mStages
    std::vector<size_t> mPeriodicStageIndices;
};