#include "pipeline_builder.h"

#include <iostream>
#include <stdexcept>

// ============================================================================
// PIPELINE BUILDER IMPLEMENTATION
// ============================================================================

PipelineBuilder::PipelineBuilder() : mOrchestrator(std::make_unique<PipelineOrchestrator>()) {}

PipelineBuilder& PipelineBuilder::addDataAcquisition(
    SharedDataManager& manager, std::shared_ptr<const IFirmware> firmware)
{
    if (!firmware)
    {
        throw std::invalid_argument("Firmware cannot be null for data acquisition stage");
    }

    auto stage = std::make_unique<DataAcquisitionStage>(manager, firmware);
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added DataAcquisition stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addTimeDomainDetection(std::unique_ptr<ITimeDomainDetector> detector)
{
    if (!detector)
    {
        throw std::invalid_argument("Time domain detector cannot be null");
    }

    auto stage = std::make_unique<TimeDomainDetectionStage>(std::move(detector));
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added TimeDomainDetection stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addFiltering(std::unique_ptr<IFrequencyDomainStrategy> filter)
{
    if (!filter)
    {
        throw std::invalid_argument("Frequency domain filter cannot be null");
    }

    auto stage = std::make_unique<FilteringStage>(std::move(filter));
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added Filtering stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addFrequencyDomainDetection(std::unique_ptr<IFrequencyDomainDetector> detector)
{
    if (!detector)
    {
        throw std::invalid_argument("Frequency domain detector cannot be null");
    }

    auto stage = std::make_unique<FrequencyDomainDetectionStage>(std::move(detector));
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added FrequencyDomainDetection stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addClassification(std::unique_ptr<ONNXModel> model, size_t spectraSize)
{
    // Classification is optional - null model is allowed
    auto stage = std::make_unique<ClassificationStage>(std::move(model), spectraSize);
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added Classification stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addDirectionEstimation(
    std::unique_ptr<GCC_PHAT> gccPhat, const Eigen::MatrixXf& cachedLS, int rank)
{
    // mCachedLeastSquares = cachedLS;
    // mHydrophoneMatrixRank = rank;

    // auto stage =
    //     std::make_unique<DirectionEstimationStage>(std::move(gccPhat), mCachedLeastSquares, mHydrophoneMatrixRank);
    auto stage = std::make_unique<DirectionEstimationStage>(std::move(gccPhat), cachedLS, rank);
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added DirectionEstimation stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addTracking(std::unique_ptr<Tracker> tracker)
{
    // Tracking is optional - null tracker is allowed
    auto stage = std::make_unique<TrackingStage>(std::move(tracker));
    mOrchestrator->addStage(std::move(stage));

    std::cout << "Added Tracking stage to pipeline" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::addCustomStage(std::unique_ptr<IProcessingStage> stage)
{
    if (!stage)
    {
        throw std::invalid_argument("Custom processing stage cannot be null");
    }

    std::cout << "Added custom stage '" << stage->getName() << "' to pipeline" << std::endl;
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::setFileOutput(const std::string& loggingDir, bool integrationTesting)
{
    if (loggingDir.empty())
    {
        throw std::invalid_argument("Logging directory cannot be empty");
    }

    auto fileHandler = std::make_unique<FileOutputHandler>(loggingDir, integrationTesting);

    if (mOutputHandler)
    {
        // If we already have an output handler, create a composite
        ensureCompositeOutputHandler();
        static_cast<CompositeOutputHandler*>(mOutputHandler.get())->addHandler(std::move(fileHandler));
    }
    else
    {
        mOutputHandler = std::move(fileHandler);
    }

    std::cout << "Added file output to directory: " << loggingDir << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::setConsoleOutput(bool verbose)
{
    auto consoleHandler = std::make_unique<ConsoleOutputHandler>(verbose);

    if (mOutputHandler)
    {
        // If we already have an output handler, create a composite
        ensureCompositeOutputHandler();
        static_cast<CompositeOutputHandler*>(mOutputHandler.get())->addHandler(std::move(consoleHandler));
    }
    else
    {
        mOutputHandler = std::move(consoleHandler);
    }

    std::cout << "Added console output (verbose: " << (verbose ? "ON" : "OFF") << ")" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::setCompositeOutput()
{
    if (!dynamic_cast<CompositeOutputHandler*>(mOutputHandler.get()))
    {
        auto composite = std::make_unique<CompositeOutputHandler>();

        if (mOutputHandler)
        {
            composite->addHandler(std::move(mOutputHandler));
        }

        mOutputHandler = std::move(composite);
    }

    return *this;
}

PipelineBuilder& PipelineBuilder::addOutputHandler(std::unique_ptr<IOutputHandler> handler)
{
    if (!handler)
    {
        throw std::invalid_argument("Output handler cannot be null");
    }

    ensureCompositeOutputHandler();
    static_cast<CompositeOutputHandler*>(mOutputHandler.get())->addHandler(std::move(handler));

    return *this;
}

PipelineBuilder& PipelineBuilder::setOutputHandler(std::unique_ptr<IOutputHandler> handler)
{
    if (!handler)
    {
        throw std::invalid_argument("Output handler cannot be null");
    }

    mOutputHandler = std::move(handler);
    std::cout << "Set custom output handler" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::setErrorHandler(std::unique_ptr<IErrorHandler> handler)
{
    if (!handler)
    {
        throw std::invalid_argument("Error handler cannot be null");
    }

    mErrorHandler = std::move(handler);
    std::cout << "Set custom error handler" << std::endl;
    return *this;
}

PipelineBuilder& PipelineBuilder::setErrorLogging(const std::string& logFile)
{
    if (logFile.empty())
    {
        throw std::invalid_argument("Log file path cannot be empty");
    }

    // Create error handler with logging
    mErrorHandler = std::make_unique<DefaultErrorHandler>(nullptr, logFile);
    std::cout << "Set error logging to file: " << logFile << std::endl;
    return *this;
}

std::unique_ptr<Pipeline> PipelineBuilder::build(
    SharedDataManager& sharedDataManager, std::chrono::seconds programRuntime,
    std::shared_ptr<ProcessingContext> processingContext)
{
    validateConfiguration();

    // Provide default output handler if none specified
    if (!mOutputHandler)
    {
        std::cout << "No output handler specified, using default file output" << std::endl;
        mOutputHandler = std::make_unique<FileOutputHandler>("./logs/");
    }

    // Provide default error handler if none specified
    if (!mErrorHandler)
    {
        mErrorHandler = std::make_unique<DefaultErrorHandler>(&sharedDataManager);
    }

    std::cout << "Building pipeline with " << mOrchestrator->getStageCount() << " stages" << std::endl;

    return std::make_unique<Pipeline>(
        sharedDataManager, std::move(mOrchestrator), std::move(mOutputHandler), std::move(mErrorHandler),
        programRuntime, processingContext);
}

void PipelineBuilder::ensureCompositeOutputHandler()
{
    if (!dynamic_cast<CompositeOutputHandler*>(mOutputHandler.get()))
    {
        auto composite = std::make_unique<CompositeOutputHandler>();

        if (mOutputHandler)
        {
            composite->addHandler(std::move(mOutputHandler));
        }

        mOutputHandler = std::move(composite);
    }
}

void PipelineBuilder::validateConfiguration() const
{
    if (mOrchestrator->getStageCount() == 0)
    {
        throw std::runtime_error("Pipeline must have at least one processing stage");
    }

    // Check if we have data acquisition stage (should be first)
    const IProcessingStage* firstStage = mOrchestrator->getStage(0);
    if (!firstStage || firstStage->getName() != "DataAcquisition")
    {
        throw std::runtime_error("Pipeline must start with a DataAcquisition stage");
    }

    std::cout << "Pipeline configuration validation passed" << std::endl;
}