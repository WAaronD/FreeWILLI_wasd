/*
DESCRIPTION:
@file main.cpp
 @brief A program for receiving and processing UDP packets in real-time.

 This program sets up a UDP listener to receive packets from a specified IP address and port.
 The received packets are stored in a shared buffer and then processed to extract and analyze data.
 The program uses two threads to handle the data receiving and processing tasks concurrently.

 Key components:
 - **UDP Listener**: Listens for incoming UDP packets, stores the received data in a buffer,
   and prints statistics about the received packets.
 - **Data Processor**: Retrieves data from the buffer, processes the data by applying filters
   and performing analysis to detect and estimate specific signal characteristics.
 - **Session and Experiment Classes**: Manage session-specific and experiment-specific details,
   including configuration settings, data buffers, and synchronization mechanisms.
 - **Multi-threading**: Uses separate threads for listening to UDP packets and processing data,
   ensuring efficient and real-time handling of incoming data.

 The program is designed to handle configuration settings dynamically based on specified
 firmware versions, adjust processing parameters accordingly, and log processed data to an output file.
 In case of errors, the program attempts to restart the listener and processing threads to maintain
 continuous operation.

 @note This program requires FFTW3 for fast Fourier transforms and Eigen for linear algebra.


EXAMPLE RUN:

Execute (datalogger simulator):
./HarpListen 192.168.7.2 1045 1240 2500 2

Execute (datalogger):
./HarpListen 192.168.100.220 50000 1240 2500 2


RESOURCES:
    debugging (core dump): https://www.youtube.com/watch?v=3T3ZDquDDVg&t=190s
    download armadillo manually: https://www.uio.no/studier/emner/matnat/fys/FYS4411/v13/guides/installing-armadillo/
*/

#include "custom_types.h"
#include "process_data.h"
#include "TDOA_estimation.h"
#include "utils.h"
//#include "pch.h"

#include "listener_thread.h"
#include "processor_thread.h"

int main(int argc, char *argv[])
{

#ifdef DEBUG
    std::cout << "Running Debug Mode" << std::endl;
#else
    std::cout << "Running Release Mode" << std::endl;
#endif

    // Declare a listening 'Session'
    Session sess;
    Experiment exp;

    sess.UDP_IP = argv[1]; // IP address of data logger or simulator
    if (sess.UDP_IP == "self")
    {
        sess.UDP_IP = "127.0.0.1";
    }

    sess.UDP_PORT = std::stoi(argv[2]);
    int firmwareVersion = std::stoi(argv[3]);
    exp.energyDetThresh = std::stod(argv[4]);
    exp.programRuntime = std::chrono::seconds(std::stoi(argv[5]));

    std::cout << "Listening to IP address " << sess.UDP_IP.c_str() << " and port " << sess.UDP_PORT << std::endl;

    // import variables according to firmware version specified
    std::cout << "Firmware version: " << firmwareVersion << std::endl;
    const std::string path = "config_files/" + std::to_string(firmwareVersion) + "_config.txt";
    if (ProcessFile(exp, path))
    {
        std::cout << "Error: Unable to open config file: " << path << std::endl;
        std::exit(1);
    }
    exp.ProcessFncPtr = ProcessSegmentInterleaved;

    exp.NUM_PACKS_DETECT = (int)(exp.TIME_WINDOW * 100000 / exp.SAMPS_PER_CHANNEL);
    exp.DATA_SEGMENT_LENGTH = exp.NUM_PACKS_DETECT * exp.SAMPS_PER_CHANNEL * exp.NUM_CHAN;

    std::cout << "HEAD_SIZE: " << exp.HEAD_SIZE << std::endl;
    std::cout << "SAMPS_PER_CHAN: " << exp.SAMPS_PER_CHANNEL << std::endl;
    std::cout << "BYTES_PER_SAMP: " << exp.BYTES_PER_SAMP << std::endl;
    std::cout << "Bytes per packet:       " << exp.REQUIRED_BYTES << std::endl;
    std::cout << "Time between packets:   " << exp.MICRO_INCR << std::endl;
    std::cout << "Number of channels:     " << exp.NUM_CHAN << std::endl;
    std::cout << "Data bytes per channel: " << exp.DATA_BYTES_PER_CHANNEL << std::endl;
    std::cout << "Detecting over a time window of " << exp.TIME_WINDOW << " seconds, using " << exp.NUM_PACKS_DETECT << " packets" << std::endl;

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

        // Destroy FFTWF objects
        fftwf_destroy_plan(exp.myFFTPlan);
        exp.myFFTPlan = nullptr;
        fftwf_destroy_plan(exp.inverseFFT);
        exp.inverseFFT = nullptr;

        sess.errorOccurred = false;
    }
}
