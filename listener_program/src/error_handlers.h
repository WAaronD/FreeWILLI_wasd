#pragma once

#include <fstream>

// #include "interfaces.h"
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
    SharedDataManager* mSharedDataManager;
    std::string mLogFile;
    bool mLogToFile;

   public:
    explicit DefaultErrorHandler(SharedDataManager* sharedDataManager = nullptr, const std::string& logFile = "");

    void handleError(const ProcessingError& error) override;

   private:
    void logContextInformation(const ProcessingContext& context);
    void logErrorToFile(const ProcessingError& error);
    void handleDataAcquisitionError(const ProcessingError& error);
    void handleClassificationError(const ProcessingError& error);
    void handleGenericError(const ProcessingError& error);
    void writeDataToCerr(const ProcessingContext& context);
};