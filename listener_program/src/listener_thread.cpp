#include "io/socket_manager_interface.h"
#include "pch.h"
#include "shared_data_manager.h"

int packetCounter = 0;  // this should only be used inside the UDPListener
                        // function, as it is not protected by a mutex

void logPacketStatistics(
    int packetCounter, int printInterval, std::chrono::steady_clock::time_point& startPacketTime, int queueSize,
    int processedPackets, int detectionCount)
{
    auto endPacketTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> durationPacketTime = endPacketTime - startPacketTime;

    // Compose log message using fixed formatting
    printf(
        "Packets rec: %d duration: %.6f queue size: %d processed packets: %d "
        "detections: %d\n",
        packetCounter, durationPacketTime.count() / printInterval, queueSize, processedPackets, detectionCount);

    startPacketTime = std::chrono::steady_clock::now();
}

void runListenerLoop(SharedDataManager& sharedDataManager, std::unique_ptr<ISocketManager>& socketManager)
{
    try
    {
        struct sockaddr_in addr;
        socklen_t addrLength = sizeof(addr);
        constexpr int printInterval = 500;

        auto startPacketTime = std::chrono::steady_clock::now();

        while (!sharedDataManager.errorOccurred)
        {
            // Receive data
            const std::vector<uint8_t>& dataBytes = socketManager->receiveData(0, (struct sockaddr*)&addr, &addrLength);

            int queueSize = sharedDataManager.pushDataToBuffer(dataBytes);

            packetCounter += 1;
            if (packetCounter % printInterval == 0)
            {
                logPacketStatistics(
                    packetCounter, printInterval, startPacketTime, queueSize, packetCounter - queueSize,
                    sharedDataManager.detectionCounter);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error occured in UDP Listener Thread: \n";
        std::stringstream msg;
        msg << e.what() << std::endl;
        std::cerr << msg.str();

        sharedDataManager.errorOccurred = true;
    }
}
