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
    bool mInitialized;

   public:
    PipelineOrchestrator();

    void addStage(std::unique_ptr<IProcessingStage> stage) override;
    bool executeStages(std::shared_ptr<ProcessingContext> context) override;
    void initializeStages(std::shared_ptr<ProcessingContext> context) override;

    size_t getStageCount() const;
    const IProcessingStage* getStage(size_t index) const;
};