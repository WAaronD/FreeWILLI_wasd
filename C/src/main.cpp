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

    Session sess;
    Experiment exp;

    InitializeSession(sess, exp, argc, argv);
    
    if (!ConfigureExperiment(exp, std::stoi(argv[3]))) {
        return 1;
    }

    while (true) 
    {
        RestartListener(sess);
        exp.programStartTime = std::chrono::system_clock::now();

        std::thread listenerThread(UdpListener, std::ref(sess), exp.PACKET_SIZE);
        std::thread processorThread(DataProcessor, std::ref(sess), std::ref(exp));

        listenerThread.join();
        processorThread.join();

        if (sess.errorOccurred) 
        {
            std::cout << "Restarting threads..." << std::endl;
        } 
        else 
        {
            std::cout << "Unknown problem occurred" << std::endl;
        }

        fftwf_destroy_plan(exp.myFFTPlan);
        exp.myFFTPlan = nullptr;
        fftwf_destroy_plan(exp.inverseFFT);
        exp.inverseFFT = nullptr;

        sess.errorOccurred = false;
    }
    return 0;
}