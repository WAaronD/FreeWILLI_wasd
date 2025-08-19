#pragma once
#include "../config_parser.h"
#include "../shared_data_manager.h"
#include "pipeline_builder.h"

class FlexiblePipelineFactory
{
   public:
    static std::unique_ptr<Pipeline> createPipeline(
        SharedDataManager& sharedDataManager, const FlexibleConfig& config, std::shared_ptr<ProcessingContext>& ctx,
        int runtimeSeconds);

   private:
    static void executeStep(
        PipelineBuilder& builder, const PipelineStep& step, SharedDataManager& sharedDataManager,
        std::shared_ptr<ProcessingContext>& ctx, const nlohmann::json& globalConfig);
};