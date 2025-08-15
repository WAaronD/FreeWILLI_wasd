#include "pipeline_orchestrator.h"

PipelineOrchestrator::PipelineOrchestrator() {}

void PipelineOrchestrator::addStage(std::unique_ptr<IProcessingStage> stage)
{
    if (!stage)
    {
        throw std::invalid_argument("Cannot add null processing stage");
    }

    // Track stages that require periodic ticking
    if (stage->requiresPeriodicTick())
    {
        mPeriodicStageIndices.push_back(mStages.size());
    }

    mStages.push_back(std::move(stage));
}
void PipelineOrchestrator::initializeStages(std::shared_ptr<ProcessingContext> context)
{
    std::cout << "Initializing " << mStages.size() << " processing stages..." << std::endl;

    for (auto& stage : mStages)
    {
        try
        {
            std::cout << "Initializing stage: " << stage->getName() << std::endl;
            stage->initialize(context);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error initializing stage " << stage->getName() << ": " << e.what() << std::endl;
            throw ProcessingError(
                stage->getName(), "Initialization failed: " + std::string(e.what()), std::current_exception(),
                *context);
        }
    }

    std::cout << "All processing stages initialized successfully." << std::endl;
}

bool PipelineOrchestrator::executeStages(std::shared_ptr<ProcessingContext> context)
{
    // Reset context for new processing cycle
    context->reset();

    // Execute each stage in sequence
    for (auto& stage : mStages)
    {
        try
        {
            // std::cout << "Executing stage"
            bool shouldContinue = stage->process(context);

            if (!shouldContinue)
            {
                // Stage indicated to stop processing (e.g., no detection)
                return false;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error in stage " << stage->getName() << ": " << e.what() << std::endl;

            throw ProcessingError(
                stage->getName(), "Stage processing failed: " + std::string(e.what()), std::current_exception(),
                *context);
        }
    }

    // All stages completed successfully
    return true;
}

size_t PipelineOrchestrator::getStageCount() const { return mStages.size(); }

const IProcessingStage* PipelineOrchestrator::getStageIndex(size_t index) const
{
    if (index >= mStages.size())
    {
        return nullptr;
    }
    return mStages[index].get();
}

const IProcessingStage* PipelineOrchestrator::getStageName(const std::string& name) const
{
        IProcessingStage* stagePointer = nullptr;
    for (auto& stage : mStages)
    {
        if (stage->getName() == name)
        {
            stagePointer = stage.get();
            break;
        }
    }
    return stagePointer;
}

void PipelineOrchestrator::tickPeriodicStages()
{
    for (size_t stageIndex : mPeriodicStageIndices)
    {
        try
        {
            mStages[stageIndex]->tick();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error in tick() for stage " << mStages[stageIndex]->getName() << ": " << e.what()
                      << std::endl;
        }
    }
}