// pipeline_factory.h
#pragma once
#include <memory>

#include "../shared_data_manager.h"
#include "../utils.h"  // for PipelineVariables
#include "pipeline.h"

/// Add new pipeline kinds here as you implement them.
enum class PipelineType
{
    AcousticProcessing
    // e.g. SpectralAnalysis, MLInference, etc.
};

/// A simple Factory for creating pipelines by type.
class PipelineFactory
{
   public:
    /// Create a pipeline of the requested type.
    /// @throws std::invalid_argument if the type isn’t supported.
    static std::unique_ptr<Pipeline> createPipeline(
        SharedDataManager& sharedDataManager, const PipelineVariables& config, int runtimeSeconds);
};
