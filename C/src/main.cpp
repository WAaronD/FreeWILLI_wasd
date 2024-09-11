/*
@file main.cpp
@brief A program for receiving and processing UDP packets in real-time.

This program sets up a UDP listener to receive packets and processes them concurrently using multi-threading. It handles configuration dynamically and logs processed data. In case of errors, it attempts to restart to maintain continuous operation.

@note Requires FFTW3 for FFT operations and Eigen for linear algebra.

Example usage:
    ./HarpListen <IP> <Port> <FirmwareVersion> <EnergyDetectionThreshold> <RuntimeSeconds>
*/

#include "custom_types.h"
#include "process_data.h"
#include "TDOA_estimation.h"
#include "utils.h"
#include "listener_thread.h"
#include "processor_thread.h"


int main(int argc, char *argv[]) 
{
    PrintMode(); // print debug or release

    // Instantiate classes for configuration and socket handling
    ExperimentConfig expConfig;
    SocketManager socketManager;
    
    // Main processing loop
    while (true) 
    {
        Session sess;
        ExperimentRuntime expRuntime;
        // Initialize the session and experiment
        InitializeSession(socketManager, expRuntime, argv);
        RestartListener(socketManager); // Reset the listener state
        
        expRuntime.programStartTime = std::chrono::system_clock::now(); // Start experiment timer

        // Create threads for listening to UDP packets and processing data
        std::thread listenerThread(UdpListener, std::ref(sess), std::ref(socketManager), expConfig.PACKET_SIZE);
        std::thread processorThread(DataProcessor, std::ref(sess), std::ref(expConfig), std::ref(expRuntime));

        // Wait for threads to finish
        listenerThread.join();
        processorThread.join();

        // Handle any errors during execution
        if (sess.errorOccurred) 
        {
            std::cout << "Restarting threads..." << std::endl;
        } 
        else 
        {
            std::cout << "Unknown problem occurred" << std::endl;
        }

        // Reset error state for the next iteration
        // sess.errorOccurred = false;
    }

    return 0;
}