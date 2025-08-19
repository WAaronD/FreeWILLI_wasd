#include "pipeline_factory.h"

std::unique_ptr<Pipeline> FlexiblePipelineFactory::createPipeline(
    SharedDataManager& sharedDataManager, const FlexibleConfig& config, std::shared_ptr<ProcessingContext>& ctx,
    int runtimeSeconds)
{
    std::cout << "Creating flexible acoustic processing pipeline...\n";

    PipelineBuilder builder;

    // Execute each pipeline step
    for (const auto& step : config.pipelineSteps)
    {
        executeStep(builder, step, sharedDataManager, ctx);
    }

    return builder.build(sharedDataManager, std::chrono::seconds(runtimeSeconds), ctx);
}

void FlexiblePipelineFactory::executeStep(
    PipelineBuilder& builder, const PipelineStep& step, SharedDataManager& sharedDataManager,
    std::shared_ptr<ProcessingContext>& ctx)
{
    const auto& params = step.params;

    if (step.type == "addDataAcquisition")
    {
        builder.addDataAcquisition(sharedDataManager, params.at("firmware").get<std::string>());
    }
    else if (step.type == "addTimeDomainDetection")
    {
        builder.addTimeDomainDetection(params.at("detector").get<std::string>(), params.at("threshold").get<float>());
    }
    else if (step.type == "addFrequencyDomainTransform")
    {
        builder.addFrequencyDomainTransform(
            params.at("strategy").get<std::string>(), params.at("filterWeightsFile").get<std::string>(), ctx,
            ctx->firmware->numChannels());
    }
    else if (step.type == "addFrequencyDomainDetection")
    {
        builder.addFrequencyDomainDetection(
            params.at("detector").get<std::string>(), params.at("threshold").get<float>());
    }
    else if (step.type == "addONNXClassification")
    {
        builder.addONNXClassification(
            params.at("modelPath").get<std::string>(), params.at("normalizationParams").get<std::string>());
    }
    else if (step.type == "addFrequencyDomainDoaEstimation")
    {
        builder.addFrequencyDomainDoaEstimation(
            params.at("receiverPositionsFile").get<std::string>(), ctx, params.at("speedOfSound").get<float>());
    }
    else if (step.type == "addTracking")
    {
        builder.addTracking(
            params.at("directory").get<std::string>(),
            std::chrono::seconds(params.at("clusteringIntervalSeconds").get<int>()),
            std::chrono::seconds(params.at("clusteringWindowSeconds").get<int>()));
    }
    else if (step.type == "addTimeDomainFilter")
    {
        builder.addTimeDomainFilter(
            params.at("filter").get<std::string>(), params.at("filterWeightsFile").get<std::string>(), ctx,
            ctx->firmware->numChannels());
    }
    else if (step.type == "setFileOutput")
    {
        builder.setFileOutput(params.at("directory").get<std::string>(), params.at("integrationTesting").get<bool>());
    }
    else if (step.type == "setConsoleOutput")
    {
        builder.setConsoleOutput(params.at("enabled").get<bool>());
    }
    else if (step.type == "setNetworkOutput")
    {
        builder.setNetworkOutput(params.at("host").get<std::string>(), params.at("port").get<int>());
    }
    else if (step.type == "setErrorHandler")
    {
        builder.setErrorHandler(params.at("logFile").get<std::string>(), sharedDataManager);
    }
    else
    {
        throw std::invalid_argument("Unknown pipeline step type: " + step.type);
    }
}