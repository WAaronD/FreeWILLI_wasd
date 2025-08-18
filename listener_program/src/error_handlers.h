#pragma once

#include "shared_data_manager.h"
#include "structs.h"

class IErrorHandler
{
   public:
    virtual ~IErrorHandler() = default;
    virtual void handleError(const ProcessingError& error) = 0;
};

class DefaultErrorHandler : public IErrorHandler
{
   private:
    SharedDataManager& mSharedDataManager;
    std::string mLogFile;
    bool mLogToFile;
    std::ofstream mFileStream;
    std::streambuf* mOldCerrBuffer = nullptr;

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

    // prevent accidental copies/moves (put this on the concrete class)
    DefaultErrorHandler(const DefaultErrorHandler&) = delete;
    DefaultErrorHandler& operator=(const DefaultErrorHandler&) = delete;
    DefaultErrorHandler(DefaultErrorHandler&&) = delete;
    DefaultErrorHandler& operator=(DefaultErrorHandler&&) = delete;
};