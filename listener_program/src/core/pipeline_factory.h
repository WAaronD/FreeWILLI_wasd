#pragma once
#include "../shared_data_manager.h"
#include "../utils.h"
#include "pipeline.h"
#include "pipeline_builder.h"

class PipelineFactory
{
   public:
    /// Create a pipeline of the requested type.
    /// @throws std::invalid_argument if the type isn't supported.
    static std::unique_ptr<Pipeline> createPipeline(
        SharedDataManager& sharedDataManager, const PipelineVariables& config, std::shared_ptr<ProcessingContext>& ctx,
        int runtimeSeconds);
};