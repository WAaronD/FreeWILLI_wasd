// main.cpp
#include <iostream>
#include <thread>

#include "core/pipeline_factory.h"
#include "io/udp_socket_manager.h"
#include "listener_thread.h"
#include "shared_data_manager.h"
#include "utils.h"  // for parseJsonConfig, printMode

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <config.json> <runtime_seconds>\n";
        return EXIT_FAILURE;
    }

    printMode();
    auto [socketVars, pipelineVars] = parseJsonConfig(argv[1]);

    std::unique_ptr<ISocketManager> socketManager = std::make_unique<UdpSocketManager>(socketVars);

    while (true)
    {
        socketManager->restartListener();
        SharedDataManager sharedDataManager;

        auto pipeline = PipelineFactory::createPipeline(sharedDataManager, pipelineVars, std::stoi(argv[2]));

        std::thread producerThread(runListenerLoop, std::ref(sharedDataManager), std::ref(socketManager));
        std::thread consumerThread(&Pipeline::process, pipeline.get());

        producerThread.join();
        consumerThread.join();
        std::cout << "Restarting threads...\n";
    }

    return 0;
}