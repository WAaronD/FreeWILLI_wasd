// pipeline_factory.cpp
#include "pipeline_factory.h"

#include <iostream>
#include <memory>
#include <stdexcept>

#include "../ML/onnx_model.h"
#include "../algorithms/detectors/frequency_domain_detectors_factory.h"
#include "../algorithms/detectors/time_domain_detectors_factory.h"
#include "../algorithms/linear_algebra_utils.h"
#include "../algorithms/localization/hydrophone_position_processing.h"
#include "../algorithms/time_domain_filters_factory.h"
#include "../firmware/firmware_factory.h"
#include "../tracker/tracker.h"
#include "pipeline_builder.h"

std::unique_ptr<Pipeline> multiChannelFrequencyDomainTracking(
    SharedDataManager& sharedDataManager, const PipelineVariables& config, int runtime)
{
    std::cout << "Creating acoustic processing pipeline...\n";

    // --- firmware ---
    auto firmware = FirmwareFactory::create(config.firmware);
    if (!firmware) throw std::runtime_error("Failed to create firmware configuration");
    std::cout << "Firmware: " << config.firmware << " (chs=" << firmware->numChannels()
              << ", sr=" << firmware->sampleRate() << ")\n";

    // --- hydrophone positions & least-squares ---
    auto positions = getHydrophoneRelativePositions(config.receiverPositionsPath);
    std::cout << "Hydrophone positions: " << positions.rows() << "×" << positions.cols() << "\n";
    auto svd = computeSvd(positions);
    auto [LS, rank] = precomputePseudoInverseAndRank(svd, config.speedOfSound);
    std::cout << "Precomputed LS, rank=" << rank << "\n";

    // --- context & filter ---
    auto ctx = std::make_shared<ProcessingContext>(firmware);
    auto filter = IFrequencyDomainTransformFactory::create(
        config.frequencyDomainStrategy, config.filterWeightsPath, ctx->channelData, firmware->numChannels());
    if (!filter) throw std::runtime_error("Failed to create frequency‐domain filter");
    std::cout << "Filter: " << config.frequencyDomainStrategy << "\n";

    // --- GCC-PHAT ---
    auto gcc = std::make_unique<GCC_PHAT>(
        filter->getPaddedLength(), filter->getFrequencyDomainData().rows(), firmware->numChannels(),
        firmware->sampleRate());
    std::cout << "GCC-PHAT ready\n";

    // --- detectors ---
    auto tdet = ITimeDomainDetectorFactory::create(config.timeDomainDetector, config.timeDomainThreshold);
    if (!tdet) throw std::runtime_error("Failed to create time-domain detector");
    auto fdet =
        IFrequencyDomainDetectorFactory::create(config.frequencyDomainDetector, config.energyDetectionThreshold);
    if (!fdet) throw std::runtime_error("Failed to create frequency-domain detector");
    std::cout << "Detectors: " << config.timeDomainDetector << ", " << config.frequencyDomainDetector << "\n";

    // --- optional ONNX / tracking ---
    auto onnx = IONNXModel::create(config);
    std::cout << (onnx ? "ONNX model loaded\n" : "No ONNX model\n");
    auto tracker = ITracker::create(config);
    std::cout << (tracker ? "Tracker loaded\n" : "No tracker\n");

    // --- build pipeline ---
    auto pipeline = PipelineBuilder()
                        .addDataAcquisition(sharedDataManager, std::move(firmware))
                        .addTimeDomainDetection(std::move(tdet))
                        .addFrequencyDomainTransform(std::move(filter))
                        .addFrequencyDomainDetection(std::move(fdet))
                        .addONNXClassification(std::move(onnx))
                        .addFrequencyDomainDoaEstimation(std::move(gcc), LS, rank)
                        .addTracking(std::move(tracker))
                        .setFileOutput(config.loggingDirectory, config.integrationTesting)
                        .setConsoleOutput(false)
                        .setErrorLogging(config.loggingDirectory + "error.log")
                        .build(sharedDataManager, std::chrono::seconds(runtime), ctx);

    std::cout << "Pipeline created successfully!\n";
    return pipeline;
}

std::unique_ptr<Pipeline> multiChannelTimeDomainClassification(
    SharedDataManager& sharedDataManager, const PipelineVariables& config, int runtime)
{
    std::cout << "Creating acoustic processing pipeline...\n";

    // --- firmware ---
    auto firmware = FirmwareFactory::create(config.firmware);
    if (!firmware) throw std::runtime_error("Failed to create firmware configuration");
    std::cout << "Firmware: " << config.firmware << " (chs=" << firmware->numChannels()
              << ", sr=" << firmware->sampleRate() << ")\n";

    // --- hydrophone positions & least-squares ---
    auto positions = getHydrophoneRelativePositions(config.receiverPositionsPath);
    std::cout << "Hydrophone positions: " << positions.rows() << "×" << positions.cols() << "\n";
    auto svd = computeSvd(positions);
    auto [LS, rank] = precomputePseudoInverseAndRank(svd, config.speedOfSound);
    std::cout << "Precomputed LS, rank=" << rank << "\n";

    // --- context & filter ---
    auto ctx = std::make_shared<ProcessingContext>(firmware);

    // --- detectors ---
    auto tdet = ITimeDomainDetectorFactory::create(config.timeDomainDetector, config.timeDomainThreshold);
    if (!tdet) throw std::runtime_error("Failed to create time-domain detector");

    auto filter = ITimeDomainFiltersFactory::create(
        config.timeDomainFilter, config.filterWeightsPath, ctx->channelData, firmware->numChannels());
    if (!filter) throw std::runtime_error("Failed to create frequency‐domain filter");

    auto tclass = ITimeDomainDetectorFactory::create("RuCCUS", 500);
    if (!tdet) throw std::runtime_error("Failed to create time-domain detector");

    // --- build pipeline ---
    auto pipeline = PipelineBuilder()
                        .addDataAcquisition(sharedDataManager, std::move(firmware))
                        .addTimeDomainDetection(std::move(tdet))
                        .addTimeDomainFilter(std::move(filter))
                        .addTimeDomainDetection(std::move(tclass))
                        .setFileOutput(config.loggingDirectory, config.integrationTesting)
                        .setConsoleOutput(false)
                        .setErrorLogging(config.loggingDirectory + "error.log")
                        .build(sharedDataManager, std::chrono::seconds(runtime), ctx);

    std::cout << "Pipeline created successfully!\n";
    return pipeline;
}

std::unique_ptr<Pipeline> PipelineFactory::createPipeline(
    SharedDataManager& sharedDataManager, const PipelineVariables& config, int runtimeSeconds)
{
    if (config.pipelineTemplate == "MultiChannelFrequencyDomainTracking")
    {
        return multiChannelFrequencyDomainTracking(sharedDataManager, config, runtimeSeconds);
    }
    if (config.pipelineTemplate == "MultiChannelTimeDomainClassification")
    {
        return multiChannelTimeDomainClassification(sharedDataManager, config, runtimeSeconds);
    }
    else
    {
        throw std::invalid_argument("PipelineFactory: unsupported pipeline type");
    }
}