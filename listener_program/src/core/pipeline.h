#pragma once

// #include <chrono>
// #include <memory>

#include "../error_handlers.h"
#include "../io/output_handlers.h"
#include "../shared_data_manager.h"
#include "core_interfaces.h"

// ============================================================================
// REFACTORED PIPELINE CLASS
// ============================================================================

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
    std::shared_ptr<ProcessingContext> mContext;
    std::unique_ptr<IPipelineOrchestrator> mOrchestrator;
    std::unique_ptr<IOutputHandler> mOutputHandler;
    std::unique_ptr<IErrorHandler> mErrorHandler;

    std::chrono::seconds mProgramRuntime;
    TimePoint mProgramStartTime;
};