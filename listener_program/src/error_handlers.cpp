#include "error_handlers.h"

#include <iomanip>
#include <iostream>
#include <sstream>

DefaultErrorHandler::DefaultErrorHandler(SharedDataManager* sharedDataManager, const std::string& logFile)
    : mSharedDataManager(sharedDataManager), mLogFile(logFile), mLogToFile(!logFile.empty())
{
}

void DefaultErrorHandler::handleError(const ProcessingError& error)
{
    std::cerr << "\n=== PROCESSING ERROR ===" << std::endl;
    std::cerr << "Stage: " << error.stageName << std::endl;
    std::cerr << "Message: " << error.errorMessage << std::endl;
    std::cerr << "========================" << std::endl;

    // Log context information
    logContextInformation(error.context);

    // Log to file if configured
    if (mLogToFile)
    {
        logErrorToFile(error);
    }

    // Stage-specific error handling
    if (error.stageName == "DataAcquisition")
    {
        handleDataAcquisitionError(error);
    }
    else if (error.stageName == "Classification")
    {
        handleClassificationError(error);
    }
    else
    {
        handleGenericError(error);
    }

    // Write debugging data to stderr
    writeDataToCerr(error.context);

    // Flag error in shared data manager
    if (mSharedDataManager)
    {
        mSharedDataManager->errorOccurred = true;
    }
}

void DefaultErrorHandler::logContextInformation(const ProcessingContext& context)
{
    std::cerr << "\n--- Context Information ---" << std::endl;
    std::cerr << "Data timestamps count: " << context.dataTimes.size() << std::endl;

    if (context.channelData.size() > 0)
    {
        std::cerr << "Channel data shape: " << context.channelData.rows() << " x " << context.channelData.cols()
                  << std::endl;
    }

    if (context.frequencyDomainData.size() > 0)
    {
        std::cerr << "Frequency domain data shape: " << context.frequencyDomainData.rows() << " x "
                  << context.frequencyDomainData.cols() << std::endl;
    }

    std::cerr << "Current result valid: " << (context.currentResult.isValid ? "YES" : "NO") << std::endl;

    /*
    if (context.tdoaVector.has_value())
    {
        std::cerr << "TDOA vector size: " << context.tdoaVector->size() << std::endl;
    }

    if (context.directionOfArrival.has_value())
    {
        std::cerr << "DOA vector size: " << context.directionOfArrival->size() << std::endl;
    }
    */

    std::cerr << "Data bytes containers: " << context.dataBytes.size() << std::endl;
    std::cerr << "-----------------------------" << std::endl;
}

void DefaultErrorHandler::logErrorToFile(const ProcessingError& error)
{
    try
    {
        std::ofstream logFile(mLogFile, std::ios::app);
        if (logFile.is_open())
        {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);

            logFile << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
            logFile << " [ERROR] Stage: " << error.stageName << " | Message: " << error.errorMessage << std::endl;

            logFile.close();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to log error to file: " << e.what() << std::endl;
    }
}

void DefaultErrorHandler::handleDataAcquisitionError(const ProcessingError& error)
{
    std::cerr << "\n--- Data Acquisition Error Recovery ---" << std::endl;
    std::cerr << "Attempting to recover from data acquisition error..." << std::endl;

    // Could implement specific recovery strategies:
    // - Clear buffers
    // - Reset firmware state
    // - Attempt reconnection
    // - Skip corrupted packets

    std::cerr << "Data acquisition error handling completed." << std::endl;
    std::cerr << "---------------------------------------" << std::endl;
}

void DefaultErrorHandler::handleClassificationError(const ProcessingError& error)
{
    std::cerr << "\n--- Classification Error Recovery ---" << std::endl;
    std::cerr << "Classification failed, system will continue without ML inference" << std::endl;

    // Classification errors are often non-fatal
    // The system can continue processing without classification

    std::cerr << "Classification error handling completed." << std::endl;
    std::cerr << "-------------------------------------" << std::endl;
}

void DefaultErrorHandler::handleGenericError(const ProcessingError& error)
{
    std::cerr << "\n--- Generic Error Handling ---" << std::endl;
    std::cerr << "Applying generic error recovery procedures..." << std::endl;

    // Generic recovery strategies:
    // - Log detailed information
    // - Attempt to continue processing
    // - Reset stage-specific state if possible

    // Attempt to re-throw and examine the original exception
    if (error.exception)
    {
        try
        {
            std::rethrow_exception(error.exception);
        }
        catch (const std::runtime_error& e)
        {
            std::cerr << "Runtime error details: " << e.what() << std::endl;
        }
        catch (const std::logic_error& e)
        {
            std::cerr << "Logic error details: " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Standard exception details: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception type" << std::endl;
        }
    }

    std::cerr << "Generic error handling completed." << std::endl;
    std::cerr << "------------------------------" << std::endl;
}

void DefaultErrorHandler::writeDataToCerr(const ProcessingContext& context)
{
    std::stringstream errorMessage;

    // Add timestamps to the error message
    errorMessage << "\n--- Debug Data Dump ---" << std::endl;
    errorMessage << "Timestamps of data causing error:" << std::endl;

    for (const auto& timestamp : context.dataTimes)
    {
        auto convertedTime =
            std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
        errorMessage << "  " << convertedTime << " μs" << std::endl;
    }

    // Add errored byte data to the error message
    errorMessage << "\nErrored bytes of last packets:" << std::endl;

    for (size_t i = 0; i < context.dataBytes.size() && i < 3; ++i)
    {  // Limit to first 3 packets
        const auto& byteArray = context.dataBytes[i];
        errorMessage << "Packet " << i << " (" << byteArray.size() << " bytes): ";

        // Show first and last few bytes
        size_t showBytes = std::min(byteArray.size(), size_t(16));
        for (size_t j = 0; j < showBytes; ++j)
        {
            errorMessage << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byteArray[j]) << " ";
        }

        if (byteArray.size() > showBytes)
        {
            errorMessage << "... (" << (byteArray.size() - showBytes) << " more bytes)";
        }

        errorMessage << std::endl;
    }

    if (context.dataBytes.size() > 3)
    {
        errorMessage << "... (" << (context.dataBytes.size() - 3) << " more packets)" << std::endl;
    }

    errorMessage << "-----------------------" << std::endl;

    // Write the composed message to the standard error stream
    std::cerr << errorMessage.str();
}