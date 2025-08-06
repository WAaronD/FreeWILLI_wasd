#pragma once

#include "../algorithms/localization/gcc_phat.h"
#include "../error_handlers.h"
#include "../io/output_handlers.h"
#include "pipeline.h"
#include "pipeline_orchestrator.h"
#include "processing_stages.h"

// Forward declarations for factory functions
class ITimeDomainDetector;
class IFrequencyDomainDetector;
class IFrequencyDomainStrategy;
class ONNXModel;
class Tracker;
class IFirmware;

// ============================================================================
// PIPELINE BUILDER (Factory Pattern)
// ============================================================================

class PipelineBuilder
{
   private:
    std::unique_ptr<PipelineOrchestrator> mOrchestrator;
    std::unique_ptr<IOutputHandler> mOutputHandler;
    std::unique_ptr<IErrorHandler> mErrorHandler;

    // Store components needed for direction estimation
    // std::optional<GCC_PHAT> mGccPhat;
    // std::optional<Eigen::MatrixXf> mCachedLeastSquares;
    // std::optional<int> mHydrophoneMatrixRank;

   public:
    PipelineBuilder();

    // Stage addition methods
    PipelineBuilder& addDataAcquisition(SharedDataManager& manager, std::shared_ptr<const IFirmware> firmware);

    PipelineBuilder& addTimeDomainDetection(std::unique_ptr<ITimeDomainDetector> detector);

    PipelineBuilder& addTimeDomainFilter(std::unique_ptr<ITimeDomainFilter> filter);

    PipelineBuilder& addFrequencyDomainTransform(std::unique_ptr<IFrequencyDomainTransform> filter);

    PipelineBuilder& addFrequencyDomainDetection(std::unique_ptr<IFrequencyDomainDetector> detector);

    PipelineBuilder& addONNXClassification(std::unique_ptr<ONNXModel> model, size_t spectraSize = 500);

    PipelineBuilder& addFrequencyDomainDoaEstimation(
        std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank);

    PipelineBuilder& addTracking(std::unique_ptr<Tracker> tracker);

    // Custom stage addition
    PipelineBuilder& addCustomStage(std::unique_ptr<IProcessingStage> stage);

    // Output and error handler configuration
    PipelineBuilder& setFileOutput(const std::string& loggingDir, bool integrationTesting = false);

    PipelineBuilder& setConsoleOutput(bool verbose = false);

    PipelineBuilder& setNetworkOutput(const std::string&, int port);

    PipelineBuilder& setCompositeOutput();

    PipelineBuilder& addOutputHandler(std::unique_ptr<IOutputHandler> handler);

    PipelineBuilder& setOutputHandler(std::unique_ptr<IOutputHandler> handler);

    PipelineBuilder& setErrorHandler(std::unique_ptr<IErrorHandler> handler);

    PipelineBuilder& setErrorLogging(const std::string& logFile);

    // Build the final pipeline
    std::unique_ptr<Pipeline> build(
        SharedDataManager& sharedDataManager, std::chrono::seconds programRuntime,
        std::shared_ptr<ProcessingContext> processingContext);

   private:
    void ensureCompositeOutputHandler();
    void validateConfiguration() const;
};