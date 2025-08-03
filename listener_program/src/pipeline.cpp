#include "pipeline.h"

#include <iostream>

#include "error_handlers.h"
#include "firmware/firmware_interface.h"
#include "pipeline_orchestrator.h"

Pipeline::Pipeline(
    SharedDataManager& sharedDataManager, std::unique_ptr<IPipelineOrchestrator> orchestrator,
    std::unique_ptr<IOutputHandler> outputHandler, std::unique_ptr<IErrorHandler> errorHandler,
    std::chrono::seconds programRuntime, std::shared_ptr<ProcessingContext> processingContext)
    : mSharedDataManager(sharedDataManager),
      mOrchestrator(std::move(orchestrator)),
      mOutputHandler(std::move(outputHandler)),
      mErrorHandler(std::move(errorHandler)),
      mProgramRuntime(programRuntime),
      mProgramStartTime(std::chrono::system_clock::now()),
      mInitialized(false),
      mContext(std::move(processingContext))
{
    // Provide default error handler if none specified
    if (!mErrorHandler)
    {
        mErrorHandler = std::make_unique<DefaultErrorHandler>(&mSharedDataManager);
    }

    // Validate required components
    if (!mOrchestrator)
    {
        throw std::invalid_argument("Pipeline orchestrator cannot be null");
    }

    if (!mOutputHandler)
    {
        throw std::invalid_argument("Output handler cannot be null");
    }
}

void Pipeline::process()
{
    try
    {
        std::cout << "Starting pipeline processing..." << std::endl;

        initializeContext();
        processLoop();

        std::cout << "Pipeline processing completed successfully." << std::endl;
    }
    catch (const ProcessingError& e)
    {
        std::cerr << "Processing error occurred, handling..." << std::endl;
        mErrorHandler->handleError(e);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Unexpected error in pipeline: " << e.what() << std::endl;
        ProcessingError error("Pipeline", e.what(), std::current_exception(), *mContext);
        mErrorHandler->handleError(error);
    }

    // Ensure output handler is properly finalized
    try
    {
        mOutputHandler->finalize();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error finalizing output handler: " << e.what() << std::endl;
    }
}

void Pipeline::initializeContext()
{
    std::cout << "Initializing pipeline context..." << std::endl;

    // Perform initial data acquisition to get firmware information and first timestamp
    performInitialDataAcquisition();

    // Initialize processing stages
    mOrchestrator->initializeStages(mContext);

    // Initialize output handler with first timestamp and channel count
    if (!mContext->dataTimes.empty() && mContext->firmware)
    {
        mOutputHandler->initialize(mContext->dataTimes[0], mContext->firmware->numChannels());
    }
    else
    {
        throw std::runtime_error("Failed to acquire initial data for pipeline initialization");
    }

    mInitialized = true;
    std::cout << "Pipeline context initialization completed." << std::endl;
}

void Pipeline::performInitialDataAcquisition()
{
    std::cout << "Performing initial data acquisition..." << std::endl;

    // Execute stages until we get a successful data acquisition
    // This is needed to initialize output files and get firmware info
    // bool dataAcquired = false;
    if (!mSharedDataManager.errorOccurred)
    {
        mOrchestrator->executeStages(mContext);
        mContext->pipelineInitialized = true;
    }
}

void Pipeline::processLoop()
{
    if (!mInitialized)
    {
        throw std::runtime_error("Pipeline not initialized. Call initializeContext() first.");
    }

    std::cout << "Starting main processing loop..." << std::endl;

    size_t iterationCount = 0;
    auto lastStatusUpdate = std::chrono::steady_clock::now();
    const auto statusInterval = std::chrono::seconds(10);

    while (!mSharedDataManager.errorOccurred && !shouldTerminate())
    {
        try
        {
            iterationCount++;

            // Execute all processing stages
            if (mOrchestrator->executeStages(mContext))
            {
                // Processing successful, handle output
                mOutputHandler->handleOutput(mContext->currentResult);
                mSharedDataManager.detectionCounter++;
            }

            mOrchestrator->tickPeriodicStages();

            // Periodic flush of output handlers
            mOutputHandler->flush();

            // Periodic status update
            auto now = std::chrono::steady_clock::now();
            if (now - lastStatusUpdate >= statusInterval)
            {
                std::cout << "Processed " << iterationCount << " iterations, " << mSharedDataManager.detectionCounter
                          << " detections" << std::endl;
                lastStatusUpdate = now;
            }
        }
        catch (const ProcessingError& e)
        {
            // Let the error handler decide how to proceed
            mErrorHandler->handleError(e);

            // If error occurred in a critical stage, we might need to break
            if (e.stageName == "DataAcquisition")
            {
                std::cerr << "Critical data acquisition error, stopping processing" << std::endl;
                break;
            }

            // For other errors, continue processing
            std::cerr << "Continuing processing after handling error in " << e.stageName << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Unexpected error in processing loop: " << e.what() << std::endl;
            ProcessingError error("ProcessingLoop", e.what(), std::current_exception(), *mContext);
            mErrorHandler->handleError(error);

            // Break on unexpected errors to prevent infinite loop
            break;
        }
    }

    // Final flush and statistics
    mOutputHandler->flush();

    std::cout << "Processing loop completed." << std::endl;
    std::cout << "Total iterations: " << iterationCount << std::endl;
    std::cout << "Total detections: " << mSharedDataManager.detectionCounter << std::endl;

    if (shouldTerminate())
    {
        std::cout << "Pipeline terminated due to runtime limit." << std::endl;
    }

    if (mSharedDataManager.errorOccurred)
    {
        std::cout << "Pipeline terminated due to error condition." << std::endl;
    }
}

bool Pipeline::shouldTerminate() const
{
    auto currentTime = std::chrono::system_clock::now();
    auto elapsedTime = currentTime - mProgramStartTime;

    bool timeExpired = (elapsedTime >= mProgramRuntime);

    if (timeExpired)
    {
        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsedTime).count();
        std::cout << "Runtime limit reached: " << elapsedSeconds << " seconds" << std::endl;
    }

    return timeExpired;
}

// Getters for testing and inspection
const IPipelineOrchestrator* Pipeline::getOrchestrator() const { return mOrchestrator.get(); }

const IOutputHandler* Pipeline::getOutputHandler() const { return mOutputHandler.get(); }

const IErrorHandler* Pipeline::getErrorHandler() const { return mErrorHandler.get(); }