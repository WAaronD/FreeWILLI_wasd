#pragma once

#include "../error_handlers.h"
#include "../io/output_handlers.h"
#include "../shared_data_manager.h"
#include "pipeline_orchestrator.h"

class Pipeline
{
   public:
    Pipeline(
        SharedDataManager& sharedDataManager, std::unique_ptr<IPipelineOrchestrator> orchestrator,
        std::unique_ptr<IOutputHandler> outputHandler, std::unique_ptr<IErrorHandler> errorHandler,
        std::chrono::seconds programRuntime, std::shared_ptr<ProcessingContext> processingContext);

    void process();

    // Getters for testing and inspection
    const IPipelineOrchestrator* getOrchestrator() const;
    const IOutputHandler* getOutputHandler() const;
    const IErrorHandler* getErrorHandler() const;

   private:
    void initializeContext();
    void processLoop();
    bool shouldTerminate() const;
    void performInitialDataAcquisition();

    SharedDataManager& mSharedDataManager;
    const std::shared_ptr<ProcessingContext> mContext;
    const std::unique_ptr<IPipelineOrchestrator> mOrchestrator;
    const std::unique_ptr<IOutputHandler> mOutputHandler;
    const std::unique_ptr<IErrorHandler> mErrorHandler;

    std::chrono::seconds mProgramRuntime;
    TimePoint mProgramStartTime;
};