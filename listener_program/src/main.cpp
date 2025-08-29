#include "core/pipeline_factory.h"
#include "firmware/firmware_factory.h"
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
    try
    {
        FlexibleConfig config = FlexibleConfigParser::parse(argv[1]);

        const std::unique_ptr<ISocketManager> socketManager = std::make_unique<UdpSocketManager>(
            config.network.at("ipAddress").get<std::string>(), config.network.at("port").get<int>());

        while (true)
        {
            socketManager->restartListener();
            SharedDataManager sharedDataManager;

            const std::shared_ptr<ProcessingContext> processingContext = std::make_shared<ProcessingContext>(
                FirmwareFactory::create(config.global.at("firmware").get<std::string>()));

            const std::unique_ptr<Pipeline> pipeline = FlexiblePipelineFactory::createPipeline(
                sharedDataManager, config, processingContext, std::stoi(argv[2]));

            std::thread producerThread(
                runListenerLoop, std::ref(sharedDataManager), std::ref(socketManager),
                config.global.at("verbose").get<bool>());
            std::thread consumerThread(&Pipeline::process, pipeline.get());

            producerThread.join();
            consumerThread.join();
            std::cout << "Restarting threads...\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return 0;
}