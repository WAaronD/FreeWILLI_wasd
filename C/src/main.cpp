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

 @note This program requires the Armadillo and SigPack libraries for matrix operations and signal processing.
       It also uses FFTW for fast Fourier transforms and includes error handling to manage runtime exceptions.


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
#include "pch.h"

using namespace std::chrono_literals;

// Global variables (used for manual testing and logging to console)
int packetCounter = 0; // this should only be used inside the UDPListener function, as it is not protected by a mutex
int detectionCounter = 0;
bool test = true;

void UdpListener(Session &sess, unsigned int PACKET_SIZE)
{
    /**
     * @brief Listens for UDP packets and processes them, storing data in a buffer.
     *
     * This function continuously listens for incoming UDP packets on a specified socket.
     * It receives data and stores the received data into a buffer (sess.dataBuffer).
     * Statistics about the received packets are printed to the console.
     *
     * @param exp Reference to an Experiment object, which contains configuration details like PACKET_SIZE.
     * @param sess Reference to a Session object, which contains session-specific details and state, such as the datagram socket and buffers.
     *
     * @throws std::runtime_error if there is an error in receiving data from the socket.
     */

    try
    {
        struct sockaddr_in addr;
        socklen_t addrLength = sizeof(addr);
        int bytesReceived;
        const int printInterval = 500;
        const int receiveSize = PACKET_SIZE + 1; // + 1 to detect if more data than expected is being received
        size_t queueSize;
        auto startPacketTime = std::chrono::steady_clock::now();
        auto endPacketTime = startPacketTime;
        std::chrono::duration<double> durationPacketTime; // stores the average amount of time (seconds) between successive UDP packets, averaged over 'printInterval' packets
        std::vector<uint8_t> dataBytes(receiveSize);

        while (!sess.errorOccurred)
        {

            bytesReceived = recvfrom(sess.datagramSocket, dataBytes.data(), receiveSize, 0, (struct sockaddr *)&addr, &addrLength);

            if (bytesReceived == -1)
                throw std::runtime_error("Error in recvfrom: bytesReceived is -1");

            dataBytes.resize(bytesReceived); // Adjust size based on actual bytes received

            sess.dataBufferLock.lock(); // give this thread exclusive rights to modify the shared dataBytes variable
            sess.dataBuffer.push(dataBytes);
            queueSize = sess.dataBuffer.size();
            sess.dataBufferLock.unlock(); // relinquish exclusive rights

            packetCounter += 1;
            if (packetCounter % printInterval == 0)
            {
                endPacketTime = std::chrono::steady_clock::now();
                durationPacketTime = endPacketTime - startPacketTime;
                std::stringstream msg; // compose message to dispatch
                msg << "Packets rec: " << packetCounter << " duration: " << std::fixed << std::setprecision(6) << durationPacketTime.count() / printInterval
                    << " queue size: " << queueSize << " processed packets: " << packetCounter - queueSize << " detections: " << detectionCounter << std::endl;
                std::cout << msg.str(); // using one instance of "<<" makes the operation atomic
                startPacketTime = std::chrono::steady_clock::now();
            }

            if (queueSize > 1000)
            { // check if buffer has grown to an unacceptable size
                throw std::runtime_error("Buffer overflowing \n");
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error occured in UDP Listener Thread: \n";

        std::stringstream msg; // compose message to dispatch
        msg << e.what() << std::endl;
        std::cerr << msg.str();

        sess.errorOccurred = true;
    }
}

void DataProcessor(Session &sess, Experiment &exp)
{
    /**
     * @brief Processes data segments from a shared buffer, performs filtering and analysis.
     *
     * This function continuously processes data segments retrieved from a shared buffer (`dataBuffer`).
     * It extracts timestamps and sample values, applies necessary adjustments and filters, and stores
     * the processed data into a segment (`dataSegment`). The processed data is then further analyzed
     * to detect pulses, apply filters, and estimate time differences and directions of arrival.
     *
     * @param exp Reference to an Experiment object containing configuration details like data segment length,
     *            number of channels, filter weights, and other processing parameters.
     * @param sess Reference to a Session object containing session-specific details and state, such as the
     *             data buffer, segment buffer, and various locks for synchronization.
     *
     * @throws std::runtime_error if there is an error in processing data, such as incorrect packet sizes
     *                            or unexpected time increments.
     */

    try
    {

        // the number of samples per channel within a dataSegment
        int channelSize = exp.DATA_SEGMENT_LENGTH / exp.NUM_CHAN;

        // Read filter weights from file
        std::vector<float> filterWeightsFloat = ReadFIRFilterFile(exp.filterWeights);

        // Declare time checking variables
        bool previousTimeSet = false;
        auto previousTime = std::chrono::time_point<std::chrono::system_clock>::min();

        // pre-allocate memory for vectors
        sess.dataSegment.reserve(exp.DATA_SEGMENT_LENGTH);
        sess.dataTimes.reserve(exp.NUM_PACKS_DETECT);

        int paddedLength = filterWeightsFloat.size() + channelSize - 1;
        int fftOutputSize = (paddedLength / 2) + 1;
        std::cout << "Padded size: " << paddedLength << std::endl;

        // Matrices for (transformed) channel data
        static Eigen::MatrixXf channelData(paddedLength, exp.NUM_CHAN);
        static Eigen::MatrixXcf savedFFTs(fftOutputSize, exp.NUM_CHAN); // save the FFT transformed channels

        /* Zero-pad filter weights to the length of the signal                     */
        std::vector<float> paddedFilterWeights(paddedLength, 0.0f);
        std::copy(filterWeightsFloat.begin(), filterWeightsFloat.end(), paddedFilterWeights.begin());

        // Create frequency domain filter
        Eigen::VectorXcf filterFreq(fftOutputSize);
        fftwf_plan fftFilter = fftwf_plan_dft_r2c_1d(paddedLength, paddedFilterWeights.data(), reinterpret_cast<fftwf_complex *>(filterFreq.data()), FFTW_ESTIMATE);
        fftwf_execute(fftFilter);
        fftwf_destroy_plan(fftFilter);

        // Container for pulling bytes from buffer (dataBuffer)
        std::vector<uint8_t> dataBytes;

        // Define the strides
        int idist = 1;  // Distance between successive elements in a channel (row-major, so it's 1)
        int odist = 1;  // Same for output
        int istride = paddedLength;  // Stride between channels in the input matrix (number of rows)
        int ostride = fftOutputSize; // Stride between channels in the output matrix

        // Create the FFTW plan with the correct strides
        exp.myFFTPlan = fftwf_plan_many_dft_r2c(1,               // Rank of the transform (1D)
                                                       &paddedLength,   // Pointer to the size of the transform
                                                       4,     // Number of transforms (channels)
                                                       channelData.data(), // Input data pointer
                                                       nullptr,         // No embedding (we're not doing multi-dimensional transforms)
                                                       idist,           // Stride between successive elements in input
                                                       istride,         // Stride between successive channels in input
                                                       reinterpret_cast<fftwf_complex*>(savedFFTs.data()), // Output data pointer
                                                       nullptr,         // No embedding
                                                       odist,           // Stride between successive elements in output
                                                       ostride,         // Stride between successive channels in output
                                                       FFTW_MEASURE);   // Flag to measure and optimize the plan

        // Fill the matrices with zeros 
        channelData.setZero();
        savedFFTs.setZero();
        
        std::vector<std::vector<float>> H = LoadHydrophonePositions(exp.receiverPositions);
        std::cout << "H:" << std::endl;
        for (const auto& coord : H){
            for (const auto& val : coord){
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }

        // set the frequency of file writes
        BufferWriter bufferWriter;
        bufferWriter._flushInterval = 1000ms;
        bufferWriter._bufferSizeThreshold = 1000; // Adjust as needed
        bufferWriter._lastFlushTime = std::chrono::steady_clock::now();

        while (!sess.errorOccurred)
        {

            sess.dataTimes.clear();
            sess.dataSegment.clear();
            sess.dataBytesSaved.clear();

            auto elapsedTime = std::chrono::system_clock::now() - exp.programStartTime;

            if (elapsedTime >= exp.programRuntime)
            {
                std::cout << "Terminating program from inside DataProcessor... duration reached" << std::endl;
                std::cout << "Flushing buffers of length: " << sess.peakAmplitudeBuffer.size() << std::endl;
                bufferWriter.write(sess, exp);
                std::exit(0);
            }

            while (sess.dataSegment.size() < exp.DATA_SEGMENT_LENGTH)
            {

                auto startLoop = std::chrono::steady_clock::now();

                sess.dataBufferLock.lock(); // give this thread exclusive rights to modify the shared dataBytes variable
                size_t queueSize = sess.dataBuffer.size();
                if (queueSize < 1)
                {
                    sess.dataBufferLock.unlock();
                    // std::cout << "Sleeping: " << std::endl;
                    std::this_thread::sleep_for(15ms);
                    continue;
                }
                else
                {
                    dataBytes = sess.dataBuffer.front();
                    sess.dataBuffer.pop();
                    sess.dataBufferLock.unlock();
                }

                sess.dataBytesSaved.push_back(dataBytes); // save bytes in case they need to be saved to a file in case of error

                // Convert byte data to floats
                auto startCDTime = std::chrono::steady_clock::now();
                ConvertData(sess.dataSegment, dataBytes, exp.DATA_SIZE, exp.HEAD_SIZE); // bytes data is decoded and appended to sess.dataSegment
                auto endCDTime = std::chrono::steady_clock::now();
                std::chrono::duration<double> durationCD = endCDTime - startCDTime;
                // std::cout << "Convert data: " << durationCD.count() << std::endl;

                auto startTimestamps = std::chrono::steady_clock::now();
                GenerateTimestamps(sess.dataTimes, dataBytes, exp.MICRO_INCR, previousTimeSet, previousTime, exp.detectionOutputFile, exp.tdoaOutputFile, exp.doaOutputFile);
                auto endTimestamps = std::chrono::steady_clock::now();
                std::chrono::duration<double> durationGenerate = endTimestamps - startTimestamps;
                // std::cout << "durationGenerate: " << durationGenerate.count() << std::endl;

                // Check if the amount of bytes in packet is what is expected
                if (dataBytes.size() != exp.PACKET_SIZE)
                {
                    std::stringstream msg; // compose message to dispatch
                    msg << "Error: incorrect number of bytes in packet: " << "PACKET_SIZE: " << exp.PACKET_SIZE << " dataBytes size: " << dataBytes.size() << std::endl;
                    throw std::runtime_error(msg.str());
                }
            }

            /*
             *   Exited inner loop - dataSegment has been filled to 'DATA_SEGMENT_LENGTH' length
             *   now apply energy detector.
             */

            auto beforePtr = std::chrono::steady_clock::now();
            exp.ProcessFncPtr(sess.dataSegment, channelData, exp.NUM_CHAN);
            auto afterPtr = std::chrono::steady_clock::now();
            std::chrono::duration<double> durationPtr = afterPtr - beforePtr;

            DetectionResult detResult = ThresholdDetect(channelData.col(0), sess.dataTimes, exp.energyDetThresh, exp.SAMPLE_RATE);

            if (detResult.maxPeakIndex < 0)
            {             // if no pulse was detected (maxPeakIndex remains -1) stop processing
                continue; // get next dataSegment; return to loop
            }

            /*
             *  Pulse detected. Now process the channels filtering, TDOA & DOA estimation.
             */

            detectionCounter++;

            auto beforeGCCW = std::chrono::steady_clock::now();
            Eigen::VectorXf resultMatrix = GCC_PHAT_FFTW(savedFFTs, exp.myFFTPlan, exp.inverseFFT, filterFreq, exp.interp, paddedLength, exp.NUM_CHAN, exp.SAMPLE_RATE);
            auto afterGCCW = std::chrono::steady_clock::now();
            std::chrono::duration<double> durationGCCW = afterGCCW - beforeGCCW;
            std::cout << "GCC time: " << durationGCCW.count() << std::endl;

            Eigen::VectorXf DOAs = DOA_EstimateVerticalArray(resultMatrix, exp.speedOfSound, exp.chanSpacing);
            std::cout << "DOAs: " << DOAs.transpose() << std::endl;

            // Write to buffers
            sess.peakAmplitudeBuffer.push_back(detResult.peakAmplitude);
            sess.peakTimesBuffer.push_back(detResult.peakTimes);
            sess.resultMatrixBuffer.push_back(resultMatrix);
            sess.DOAsBuffer.push_back(DOAs);

            if (sess.peakAmplitudeBuffer.size() >= bufferWriter._bufferSizeThreshold)
            { //|| currentTime - lastFlushTime >= FLUSH_INTERVAL) {
                std::cout << "Flushing buffers of length: " << sess.peakAmplitudeBuffer.size() << std::endl;
                bufferWriter.write(sess, exp);
            }
        }
    }
    catch (const GCC_Value_Error &e)
    {

        std::stringstream msg; // compose message to dispatch
        msg << e.what() << std::endl;
        std::cerr << msg.str();

        try
        {
            WriteDataToCerr(sess.dataTimes, sess.dataBytesSaved);
        }
        catch (...)
        {
            std::cerr << "failed to write data to cerr \n";
        }
        sess.errorOccurred = true;
    }
    catch (const std::ios_base::failure &e)
    {
        std::stringstream msg; // compose message to dispatch
        msg << e.what() << std::endl;
        std::cerr << msg.str();
        std::exit(1);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error occured in data processor thread: \n";

        std::stringstream msg; // compose message to dispatch
        msg << e.what() << std::endl;
        std::cerr << msg.str();

        try
        {
            WriteDataToCerr(sess.dataTimes, sess.dataBytesSaved);
        }
        catch (...)
        {
            std::cerr << "failed to write data to cerr \n";
        }
        sess.errorOccurred = true;
        std::cerr << "End of catch statement\n";
    }
}

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
    std::cout << "IP " << sess.UDP_IP << std::endl;
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
