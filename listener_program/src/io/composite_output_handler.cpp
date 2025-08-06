#include "output_handlers.h"

void CompositeOutputHandler::addHandler(std::unique_ptr<IOutputHandler> handler)
{
    mHandlers.push_back(std::move(handler));
}

void CompositeOutputHandler::initialize(const TimePoint& timestamp, int numChannels)
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->initialize(timestamp, numChannels);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error initializing output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::handleOutput(const DetectionResult& result)
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->handleOutput(result);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error in output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::flush()
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->flush();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error flushing output handler: " << e.what() << std::endl;
        }
    }
}

void CompositeOutputHandler::finalize()
{
    for (auto& handler : mHandlers)
    {
        try
        {
            handler->finalize();
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error finalizing output handler: " << e.what() << std::endl;
        }
    }
}
