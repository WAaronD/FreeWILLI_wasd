#pragma once

#include "../algorithms/localization/gcc_phat.h"
#include "../error_handlers.h"
#include "../io/output_handlers.h"
#include "pipeline.h"
#include "pipeline_orchestrator.h"
#include "processing_stages.h"

// Forward declarations
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;
class IFirmware;

class PipelineBuilder
{
   public:
    PipelineBuilder();

    // Stage addition methods with object creation
    PipelineBuilder& addDataAcquisition(SharedDataManager& manager, const std::string& firmwareType);

    PipelineBuilder& addTimeDomainDetection(const std::string& detectorType, double threshold);

    PipelineBuilder& addTimeDomainFilter(
        const std::string& filterType, const std::string& weightsPath, const std::shared_ptr<ProcessingContext>& ctx,
        int numChannels);

    PipelineBuilder& addFrequencyDomainTransform(
        const std::string& strategyType, const std::string& weightsPath, const std::shared_ptr<ProcessingContext>& ctx,
        int numChannels);

    PipelineBuilder& addFrequencyDomainDetection(const std::string& detectorType, double threshold);

    PipelineBuilder& addONNXClassification(const std::string& modelPath, const std::string& scalerParamsPath);

    PipelineBuilder& addFrequencyDomainDoaEstimation(
        const std::string& receiverPositionsPath, const std::shared_ptr<ProcessingContext>& ctx, float speedOfSound);

    PipelineBuilder& addTracking(
        const std::string& outputDirectory, std::chrono::seconds clusteringFrequency,
        std::chrono::seconds clusteringWindow);

    // Custom stage addition
    PipelineBuilder& addCustomStage(std::unique_ptr<IProcessingStage> stage);

    // Output and error handler configuration
    PipelineBuilder& setFileOutput(const std::string& loggingDir, bool integrationTesting = false);

    PipelineBuilder& setConsoleOutput(bool verbose = false);

    PipelineBuilder& setNetworkOutput(const std::string& ip, int port);

    PipelineBuilder& setCompositeOutput();

    PipelineBuilder& addOutputHandler(std::unique_ptr<IOutputHandler> handler);

    PipelineBuilder& setOutputHandler(std::unique_ptr<IOutputHandler> handler);

    PipelineBuilder& setErrorHandler(const std::string& outputFile, SharedDataManager& sharedDataManager);

    // Build the final pipeline
    std::unique_ptr<Pipeline> build(
        SharedDataManager& sharedDataManager, std::chrono::seconds programRuntime,
        const std::shared_ptr<ProcessingContext>& processingContext);

   private:
    void ensureCompositeOutputHandler();
    void validateConfiguration() const;

    std::unique_ptr<PipelineOrchestrator> mOrchestrator;
    std::unique_ptr<IOutputHandler> mOutputHandler;
    std::unique_ptr<IErrorHandler> mErrorHandler;
    std::shared_ptr<const IFirmware> mFirmware;  // Store firmware for later stages
};