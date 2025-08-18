#include "pipeline_factory.h"

std::unique_ptr<Pipeline> PipelineFactory::createPipeline(
    SharedDataManager& sharedDataManager, const PipelineVariables& config, std::shared_ptr<ProcessingContext>& ctx,
    int runtimeSeconds)
{
    std::cout << "Creating acoustic processing pipeline...\n";

    if (config.pipelineTemplate == "MultiChannelFrequencyDomainTracking")
    {
        return PipelineBuilder()
            .addDataAcquisition(sharedDataManager, config.firmware)
            .addTimeDomainDetection(config.timeDomainDetector, config.timeDomainThreshold)
            .addFrequencyDomainTransform(
                config.frequencyDomainStrategy, config.filterWeightsPath, ctx, ctx->firmware->numChannels())
            .addFrequencyDomainDetection(config.frequencyDomainDetector, config.energyDetectionThreshold)
            .addONNXClassification(config)
            .addFrequencyDomainDoaEstimation(config.receiverPositionsPath, ctx, config.speedOfSound)
            .addTracking(config)
            .setFileOutput(config.loggingDirectory, config.integrationTesting)
            .setConsoleOutput(false)
            .setErrorHandler("error.log")
            .build(sharedDataManager, std::chrono::seconds(runtimeSeconds), ctx);
    }
    else if (config.pipelineTemplate == "MultiChannelTimeDomainClassification")
    {
        return PipelineBuilder()
            .addDataAcquisition(sharedDataManager, config.firmware)
            .addTimeDomainDetection(config.timeDomainDetector, config.timeDomainThreshold)
            .addTimeDomainFilter(config.timeDomainFilter, config.filterWeightsPath, ctx, ctx->firmware->numChannels())
            .addTimeDomainDetection("RuCCUS", 500)
            .setFileOutput(config.loggingDirectory, config.integrationTesting)
            .setConsoleOutput(false)
            .setNetworkOutput("127.0.0.1", 55001)
            .setErrorHandler("error.log")
            .build(sharedDataManager, std::chrono::seconds(runtimeSeconds), ctx);
    }
    else
    {
        throw std::invalid_argument("PipelineFactory: unsupported pipeline type");
    }
}