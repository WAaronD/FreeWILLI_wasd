#pragma once

#include "processing_context_struct.h"
#include "shared_data_manager.h"

struct ProcessingError
{
    std::string stageName;
    std::string errorMessage;
    std::exception_ptr exception;
    ProcessingContext context;

    ProcessingError(const std::string& stage, const std::string& message) : stageName(stage), errorMessage(message) {}

    ProcessingError(
        const std::string& stage, const std::string& message, std::exception_ptr ex, const ProcessingContext& ctx)
        : stageName(stage), errorMessage(message), exception(ex), context(ctx)
    {
    }
};

class IErrorHandler
{
   public:
    virtual ~IErrorHandler() = default;
    virtual void handleError(const ProcessingError& error) = 0;
};

class DefaultErrorHandler : public IErrorHandler
{
   public:
    explicit DefaultErrorHandler(SharedDataManager& sharedDataManager, const std::string& logFile);
    ~DefaultErrorHandler() override;

    void handleError(const ProcessingError& error) override;

   private:
    void logContextInformation(const ProcessingContext& context);
    void handleDataAcquisitionError(const ProcessingError& error);
    void handleClassificationError(const ProcessingError& error);
    void handleGenericError(const ProcessingError& error);
    void writeDataToCerr(const ProcessingContext& context);

    DefaultErrorHandler(const DefaultErrorHandler&) = delete;
    DefaultErrorHandler& operator=(const DefaultErrorHandler&) = delete;
    DefaultErrorHandler(DefaultErrorHandler&&) = delete;
    DefaultErrorHandler& operator=(DefaultErrorHandler&&) = delete;

    SharedDataManager& mSharedDataManager;
    std::string mLogFile;
    bool mLogToFile;
    std::ofstream mFileStream;
    std::streambuf* mOldCerrBuffer = nullptr;
};