#include "io/udp_socket_manager.h"
#include "listener_thread.h"
// #include "pipeline.h"
#include "ML/onnx_model.h"
#include "algorithms/fir_filter_factory.h"
#include "algorithms/frequency_domain_detectors_factory.h"
#include "algorithms/hydrophone_position_processing.h"
#include "algorithms/linear_algebra_utils.h"
#include "algorithms/time_domain_detectors_factory.h"
#include "firmware/firmware_factory.h"
#include "pipeline_builder.h"
#include "shared_data_manager.h"
#include "tracker/tracker.h"
#include "utils.h"

std::unique_ptr<Pipeline> createAcousticProcessingPipeline(
    SharedDataManager& sharedDataManager, const PipelineVariables& config, int runtime)
{
    std::cout << "Creating acoustic processing pipeline..." << std::endl;

    // Create firmware configuration
    auto firmware = FirmwareFactory::create(config.firmware);
    if (!firmware)
    {
        throw std::runtime_error("Failed to create firmware configuration");
    }

    std::cout << "Firmware created: " << config.firmware << std::endl;
    std::cout << "Channels: " << firmware->numChannels() << ", Sample rate: " << firmware->sampleRate() << std::endl;

    // Prepare hydrophone positions and direction estimation components
    Eigen::MatrixXf hydrophonePositions = getHydrophoneRelativePositions(config.receiverPositionsPath);
    std::cout << "Loaded hydrophone positions: " << hydrophonePositions.rows() << "x" << hydrophonePositions.cols()
              << std::endl;

    auto svdDecomposition = computeSvd(hydrophonePositions);
    auto [cachedLeastSquares, rank] = precomputePseudoInverseAndRank(svdDecomposition, config.speedOfSound);

    std::cout << "Precomputed least squares matrix, rank: " << rank << std::endl;

    // Create channel data matrix for filter initialization
    // Eigen::MatrixXf channelData = Eigen::MatrixXf::Zero(firmware->numChannels(), firmware->channelSize());
    // ProcessingContext processingContext(firmware);
    auto ctx = std::make_shared<ProcessingContext>(firmware);
    // Create frequency domain filter
    auto filter = IFrequencyDomainStrategyFactory::create(
        config.frequencyDomainStrategy, config.filterWeightsPath, ctx->channelData, firmware->numChannels());

    if (!filter)
    {
        throw std::runtime_error("Failed to create frequency domain filter");
    }

    std::cout << "Created frequency domain filter: " << config.frequencyDomainStrategy << std::endl;

    // Create GCC-PHAT processor for direction estimation
    auto gccPhat = std::make_unique<GCC_PHAT>(
        filter->getPaddedLength(), filter->getFrequencyDomainData().rows(), firmware->numChannels(),
        firmware->sampleRate());

    std::cout << "Created GCC-PHAT processor" << std::endl;

    // Create detectors
    auto timeDomainDetector = ITimeDomainDetectorFactory::create(config.timeDomainDetector, config.timeDomainThreshold);

    if (!timeDomainDetector)
    {
        throw std::runtime_error("Failed to create time domain detector");
    }

    auto frequencyDomainDetector =
        IFrequencyDomainDetectorFactory::create(config.frequencyDomainDetector, config.energyDetectionThreshold);

    if (!frequencyDomainDetector)
    {
        throw std::runtime_error("Failed to create frequency domain detector");
    }

    std::cout << "Created detectors: " << config.timeDomainDetector << ", " << config.frequencyDomainDetector
              << std::endl;

    // Create optional components
    auto onnxModel = IONNXModel::create(config);  // May return nullptr if not configured
    auto tracker = ITracker::create(config);  // May return nullptr if not configured

    if (onnxModel)
    {
        std::cout << "Created ONNX classification model" << std::endl;
    }
    else
    {
        std::cout << "No ONNX model configured" << std::endl;
    }

    if (tracker)
    {
        std::cout << "Created tracker" << std::endl;
    }
    else
    {
        std::cout << "No tracker configured" << std::endl;
    }

    // Build the pipeline using the builder pattern
    auto pipeline = PipelineBuilder()
                        .addDataAcquisition(sharedDataManager, std::move(firmware))
                        .addTimeDomainDetection(std::move(timeDomainDetector))
                        .addFiltering(std::move(filter))
                        .addFrequencyDomainDetection(std::move(frequencyDomainDetector))
                        .addClassification(std::move(onnxModel))
                        .addDirectionEstimation(std::move(gccPhat), cachedLeastSquares, rank)
                        .addTracking(std::move(tracker))
                        .setFileOutput(config.loggingDirectory, config.integrationTesting)
                        .setConsoleOutput(false)  // Non-verbose console output
                        .setErrorLogging(config.loggingDirectory + "error.log")
                        .build(sharedDataManager, std::chrono::seconds(runtime), ctx);

    std::cout << "Pipeline created successfully!" << std::endl;
    return pipeline;
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <config_files/config.json> <runtime_duration>" << std::endl;
        return EXIT_FAILURE;
    }

    printMode();

    auto [socketVariables, pipelineVars] = parseJsonConfig(std::string(argv[1]));

    std::unique_ptr<ISocketManager> socketManager = std::make_unique<UdpSocketManager>(socketVariables);

    while (true)
    {
        socketManager->restartListener();

        SharedDataManager sharedDataManager;
        // OutputManager outputManager(
        //     std::chrono::seconds(std::stoi(argv[2])), pipelineVars.integrationTesting,
        //     pipelineVars.loggingDirectory);

        // Pipeline pipeline(outputManager, sharedDataManager, pipelineVars);

        // Create and run the main pipeline
        auto pipeline = createAcousticProcessingPipeline(sharedDataManager, pipelineVars, std::stoi(argv[2]));

        // Create threads for listening for incoming data packets and processing data
        std::thread producerThread(runListenerLoop, std::ref(sharedDataManager), std::ref(socketManager));
        // std::thread consumerThread(&Pipeline::process, &pipeline);
        std::thread consumerThread(&Pipeline::process, pipeline.get());

        // Wait for threads to finish
        producerThread.join();
        consumerThread.join();

        std::cout << "Restarting threads..." << std::endl;
    }
    return 0;
}
