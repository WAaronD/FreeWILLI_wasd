#include "pipeline_builder.h"

#include "../ML/onnx_model.h"
#include "../algorithms/detectors/frequency_domain_detectors_factory.h"
#include "../algorithms/detectors/time_domain_detectors_factory.h"
#include "../algorithms/linear_algebra_utils.h"
#include "../algorithms/localization/hydrophone_position_processing.h"
#include "../algorithms/time_domain_filters_factory.h"
#include "../firmware/firmware_factory.h"
#include "../tracker/tracker.h"

PipelineBuilder::PipelineBuilder() : mOrchestrator(std::make_unique<PipelineOrchestrator>()) {}

PipelineBuilder& PipelineBuilder::addDataAcquisition(SharedDataManager& manager)
{
    std::cout << "Adding DataAcquisition stage to pipeline" << std::endl;

    auto stage = std::make_unique<DataAcquisitionStage>(manager);
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addTimeDomainDetection(const nlohmann::json& params)
{
    std::cout << "Adding TimeDomainDetection stage to pipeline" << std::endl;
    auto detector = ITimeDomainDetectorFactory::create(params);

    if (!detector)
    {
        throw std::runtime_error("Failed to create time-domain detector");
    }
    auto stage = std::make_unique<TimeDomainDetectionStage>(std::move(detector));
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addTimeDomainFilter(
    const std::string& filterType, const std::string& weightsPath, const std::shared_ptr<ProcessingContext>& ctx,
    int numChannels)
{
    std::cout << "Adding TimeDomainFiltering stage to pipeline" << std::endl;
    auto filter = ITimeDomainFiltersFactory::create(filterType, weightsPath, ctx->channelData, numChannels);
    if (!filter)
    {
        throw std::runtime_error("Failed to create time-domain filter: " + filterType);
    }

    auto stage = std::make_unique<TimeDomainFilteringStage>(std::move(filter));
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addFrequencyDomainTransform(
    const std::string& strategyType, const std::string& weightsPath, const std::shared_ptr<ProcessingContext>& ctx,
    int numChannels)
{
    std::cout << "Adding FrequencyDomainTransform stage to pipeline" << std::endl;
    auto filter = IFrequencyDomainTransformFactory::create(strategyType, weightsPath, ctx->channelData, numChannels);

    ctx->frequencyDomainData = filter->getFrequencyDomainData();

    if (!filter)
    {
        throw std::runtime_error("Failed to create frequency-domain filter: " + strategyType);
    }

    auto stage = std::make_unique<FrequencyDomainFilteringStage>(std::move(filter));
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addFrequencyDomainDetection(const nlohmann::json& params)
{
    std::cout << "Adding FrequencyDomainDetection stage to pipeline" << std::endl;
    auto detector = IFrequencyDomainDetectorFactory::create(params);

    auto stage = std::make_unique<FrequencyDomainDetectionStage>(std::move(detector));
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addONNXClassification(
    const std::string& modelPath, const std::string& scalerParamsPath)
{
    std::cout << "Adding ONNXClassification stage to pipeline" << std::endl;
    std::unique_ptr<ONNXModel> model = nullptr;

    model = IONNXModel::create(modelPath, scalerParamsPath);

    auto stage = std::make_unique<ONNXClassificationStage>(std::move(model), 500);
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addFrequencyDomainDoaEstimation(
    const std::string& receiverPositionsPath, const std::shared_ptr<ProcessingContext>& ctx, float speedOfSound)
{
    std::cout << "Adding FrequencyDomainDoaEstimation stage to pipeline" << std::endl;
    auto gccPhat = std::make_unique<GCC_PHAT>(
        ctx->channelData.cols(), ctx->frequencyDomainData.rows(), ctx->firmware->numChannels(),
        ctx->firmware->sampleRate());

    if (!gccPhat)
    {
        throw std::runtime_error("Failed to create GCC_PHAT");
    }

    auto positions = getHydrophoneRelativePositions(receiverPositionsPath);
    std::cout << "    Hydrophone positions: " << positions.rows() << "×" << positions.cols() << "\n";
    auto svd = computeSvd(positions);
    auto [LS, rank] = precomputePseudoInverseAndRank(svd, speedOfSound);
    std::cout << "    Precomputed LS, rank=" << rank << "\n";

    auto stage = std::make_unique<DirectionEstimationStage>(std::move(gccPhat), LS, rank);
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addTracking(
    const std::string& outputDirectory, std::chrono::seconds clusteringFrequency, std::chrono::seconds clusteringWindow)
{
    std::cout << "Adding Tracking stage to pipeline" << std::endl;
    std::unique_ptr<Tracker> tracker = nullptr;

    tracker = ITracker::create(outputDirectory, clusteringFrequency, clusteringWindow);

    auto stage = std::make_unique<TrackingStage>(std::move(tracker));
    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::addCustomStage(std::unique_ptr<IProcessingStage> stage)
{
    std::cout << "Adding custom stage '" << stage->getName() << "' to pipeline" << std::endl;
    if (!stage)
    {
        throw std::invalid_argument("Custom processing stage cannot be null");
    }

    mOrchestrator->addStage(std::move(stage));

    return *this;
}

PipelineBuilder& PipelineBuilder::setFileOutput(const std::string& loggingDir, bool integrationTesting)
{
    std::cout << "Adding file output to directory: " << loggingDir << std::endl;
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

    return *this;
}

PipelineBuilder& PipelineBuilder::setConsoleOutput(bool verbose)
{
    std::cout << "Adding console output (verbose: " << (verbose ? "ON" : "OFF") << ")" << std::endl;
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

    return *this;
}

PipelineBuilder& PipelineBuilder::setNetworkOutput(const std::string& ip, int port)
{
    std::cout << "Adding network output at IP address: " << ip << " " << port << std::endl;
    if (ip.empty())
    {
        throw std::invalid_argument("IP address for network output cannot be empty");
    }

    auto networkHandler = std::make_unique<NetworkOutputHandler>(ip, port);

    if (mOutputHandler)
    {
        // If we already have an output handler, create a composite
        ensureCompositeOutputHandler();
        static_cast<CompositeOutputHandler*>(mOutputHandler.get())->addHandler(std::move(networkHandler));
    }
    else
    {
        mOutputHandler = std::move(networkHandler);
    }

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

PipelineBuilder& PipelineBuilder::setErrorHandler(const std::string& outputFile, SharedDataManager& SharedDataManager)
{
    auto errorHandler = std::make_unique<DefaultErrorHandler>(SharedDataManager, outputFile);

    mErrorHandler = std::move(errorHandler);
    std::cout << "Set custom error handler" << std::endl;
    return *this;
}

std::unique_ptr<Pipeline> PipelineBuilder::build(
    SharedDataManager& sharedDataManager, std::chrono::seconds programRuntime,
    const std::shared_ptr<ProcessingContext>& processingContext)
{
    std::cout << "Building pipeline with " << mOrchestrator->getStageCount() << " stages" << std::endl;
    validateConfiguration();

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
    const IProcessingStage* firstStage = mOrchestrator->getStageIndex(0);
    if (!firstStage || firstStage->getName() != "Data Acquisition")
    {
        throw std::runtime_error("Pipeline must start with a DataAcquisition stage");
    }

    std::cout << "Pipeline configuration validation passed" << std::endl;
}