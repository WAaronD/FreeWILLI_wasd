#include "io/udp_socket_manager.h"
#include "listener_thread.h"
#include "pipeline.h"
#include "shared_data_manager.h"
#include "utils.h"

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <config_files/config.json> <runtime_duration>" << std::endl;
        return EXIT_FAILURE;
    }

    printMode();

    auto [socketVariables, pipelineVars] = parseJsonConfig(std::string(argv[1]));

    std::unique_ptr<ISocketManager> socketManager = std::make_unique<UdpSocketManager>(socketVariables);

    while (true)
    {
        socketManager->restartListener();

        SharedDataManager sharedDataManager;
        OutputManager outputManager(
            std::chrono::seconds(std::stoi(argv[2])), pipelineVars.integrationTesting, pipelineVars.loggingDirectory);

        Pipeline pipeline(outputManager, sharedDataManager, pipelineVars);

        // Create threads for listening for incoming data packets and processing data
        std::thread producerThread(runListenerLoop, std::ref(sharedDataManager), std::ref(socketManager));
        std::thread consumerThread(&Pipeline::process, &pipeline);

        // Wait for threads to finish
        producerThread.join();
        consumerThread.join();

        std::cout << "Restarting threads..." << std::endl;
    }
    return 0;
}
