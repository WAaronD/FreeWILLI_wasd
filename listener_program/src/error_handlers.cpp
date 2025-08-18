#include "error_handlers.h"

DefaultErrorHandler::DefaultErrorHandler(SharedDataManager& sharedDataManager, const std::string& logFile)
    : mSharedDataManager(sharedDataManager), mLogFile(logFile), mLogToFile(!logFile.empty())
{
    if (mLogToFile)
    {
        mFileStream.open(mLogFile, std::ios::app);
        if (mFileStream.is_open())
        {
            mOldCerrBuffer = std::cerr.rdbuf(mFileStream.rdbuf());
        }
        else
        {
            mLogToFile = false;  // failed to open; leave cerr as-is
        }
    }
}

DefaultErrorHandler::~DefaultErrorHandler()
{
    std::cerr.flush();
    if (mOldCerrBuffer)
    {  // only restore if we changed it
        std::cerr.rdbuf(mOldCerrBuffer);
        mOldCerrBuffer = nullptr;
    }
    if (mFileStream.is_open())
    {
        mFileStream.flush();
        mFileStream.close();
    }
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
    auto now = std::chrono::system_clock::now();
    auto timeT = std::chrono::system_clock::to_time_t(now);

    std::cerr << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S");
    std::cerr << " [ERROR] Stage: " << error.stageName << " | Message: " << error.errorMessage << std::endl;

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
    std::cout << "setting mSharedDataManager->errorOccurred = true " << std::endl;
    mSharedDataManager.errorOccurred = true;
    std::cout << "FAILED: setting mSharedDataManager->errorOccurred = true " << std::endl;
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
    std::cerr << "Data bytes containers: " << context.dataBytes.size() << std::endl;
    std::cerr << "-----------------------------" << std::endl;
}

void DefaultErrorHandler::handleDataAcquisitionError(const ProcessingError& error)
{
    std::cerr << "\n--- Data Acquisition Error Recovery ---" << std::endl;
    std::cerr << "Error message: " << error.errorMessage << std::endl;

    std::cerr << "Data acquisition error handling completed." << std::endl;
    std::cerr << "---------------------------------------" << std::endl;
}

void DefaultErrorHandler::handleClassificationError(const ProcessingError& error)
{
    std::cerr << "\n--- Classification Error Recovery ---" << std::endl;
    std::cerr << "Error message: " << error.errorMessage << std::endl;

    std::cerr << "Classification error handling completed." << std::endl;
    std::cerr << "-------------------------------------" << std::endl;
}

void DefaultErrorHandler::handleGenericError(const ProcessingError& error)
{
    std::cerr << "\n--- Generic Error Handling ---" << std::endl;
    std::cerr << "Error message: " << error.errorMessage << std::endl;

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

    for (size_t i = 0; i < context.dataBytes.size(); ++i)
    {  // Limit to first 3 packets
        const auto& byteArray = context.dataBytes[i];
        errorMessage << "Packet " << i << " (" << byteArray.size() << " bytes): ";

        // Show first and last few bytes
        // size_t showBytes = std::min(byteArray.size(), size_t(16));
        for (size_t j = 0; j < byteArray.size(); ++j)
        {
            errorMessage << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byteArray[j]) << " ";
        }

        /*
        if (byteArray.size() > showBytes)
        {
            errorMessage << "... (" << (byteArray.size() - showBytes) << " more bytes)";
        }
        */

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