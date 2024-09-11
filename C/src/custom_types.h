#pragma once

#include "pch.h"

// forward declare processing function. Needed for function pointer in ExperimentConfig
void ProcessSegmentInterleaved(std::span<float> data, Eigen::MatrixXf &channelData, const int NUM_CHAN);

using TimePoint = std::chrono::system_clock::time_point;
using namespace std::chrono_literals;

class GCC_Value_Error : public std::runtime_error
{
public:
    GCC_Value_Error(const std::string &message) : std::runtime_error(message) {}
};

class ExperimentConfig
{
public:
    // UDP packet information
    static constexpr int HEAD_SIZE         = 12;   // packet head size (bytes)
    static constexpr int NUM_CHAN          = 4;    // number of channels per packet
    static constexpr int SAMPS_PER_CHANNEL = 124;  // samples per packet per channel
    static constexpr int BYTES_PER_SAMP    = 2;    // bytes per sample
    static constexpr int MICRO_INCR        = 1240; // time between packets
    static constexpr int SAMPLE_RATE       = 1e5;  // sample rate in Hz
    
    static constexpr int DATA_SIZE = SAMPS_PER_CHANNEL * NUM_CHAN * BYTES_PER_SAMP;   // packet data size (bytes)
    static constexpr int PACKET_SIZE = DATA_SIZE + HEAD_SIZE;                         // packet size (bytes)
    static constexpr int REQUIRED_BYTES = DATA_SIZE + HEAD_SIZE;
    static constexpr int DATA_BYTES_PER_CHANNEL = SAMPS_PER_CHANNEL * BYTES_PER_SAMP; // number of data bytes per channel (REQUIRED_BYTES - 12) / 4 channels


    static constexpr float TIME_WINDOW = 0.01; // fraction of a second to consider when performing cross correlation 
    static constexpr int   NUM_PACKS_DETECT = static_cast<int>(TIME_WINDOW * 100000 / SAMPS_PER_CHANNEL);
    static constexpr int   DATA_SEGMENT_LENGTH = NUM_PACKS_DETECT * SAMPS_PER_CHANNEL * NUM_CHAN;
    static constexpr float speedOfSound = 1482.965459;
    static constexpr int   interp = 1;

    // std::vector<float> chanSpacing = {1.0, 2.0, 3.0, 1.0, 2.0, 1.0};

    static constexpr const char* filterWeights = "filters/highpass_taps@101_cutoff@20k_window@hamming_fs@100k.txt";
    static constexpr const char* receiverPositions = "../Data/SOCAL_H_72_HS_harp4chPar_recPos.txt";
    
    
    const std::function<void(std::span<float>, Eigen::MatrixXf &, unsigned int)> ProcessFncPtr = ProcessSegmentInterleaved;
     
};

class ExperimentRuntime {
public:
    
    std::string          detectionOutputFile = ""; // Define at runtime
    std::chrono::seconds programRuntime;
    TimePoint            programStartTime; // placeholder value
    float                energyDetThresh = 100.0f; // 28.0f; // energy detector threshold - 2500.0 is default
    
    fftwf_plan forwardFFT = nullptr;
    fftwf_plan inverseFFT = nullptr;

    // Constructor to initialize plans, runtime-specific methods here
    ExperimentRuntime() {
        // Initialize FFT plans dynamically, if needed
    }

    ~ExperimentRuntime() {
        if (forwardFFT) fftwf_destroy_plan(forwardFFT);
        if (inverseFFT) fftwf_destroy_plan(inverseFFT);
    }
};

class SocketManager{
public:
    int         datagramSocket = socket(AF_INET, SOCK_DGRAM, 0); // udp socket
    int         UDP_PORT;                                        // Listening port
    std::string UDP_IP; // IP address of data logger or simulator

};

//(focuses on managing shared resources and thread safety):
class Session {
public:
    std::atomic<bool> errorOccurred = false;
    std::queue<std::vector<uint8_t>> dataBuffer;
    std::vector<std::vector<uint8_t>> dataBytesSaved;
    std::vector<float> dataSegment;
    std::vector<std::chrono::system_clock::time_point> dataTimes;
    std::mutex dataBufferLock;
    int detectionCounter = 0;
    std::vector<Eigen::VectorXf> Buffer;
    std::vector<std::chrono::system_clock::time_point> peakTimesBuffer;

    Session() {
        // Initialize other runtime-specific variables as needed
    }

    // Add methods for buffer management
    int pushDataToBuffer(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(dataBufferLock);
        dataBuffer.push(data);
        return dataBuffer.size();
    }

    std::vector<uint8_t> popDataFromBuffer() {
        std::vector<uint8_t> data;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(dataBufferLock);
                if (!dataBuffer.empty()) {
                    data = dataBuffer.front();
                    dataBuffer.pop();
                    return data;
                }
            }
            // Sleep for 15ms before trying again
            std::this_thread::sleep_for(std::chrono::milliseconds(15ms));
        }
    }
};


struct DetectionResult
{
    int minPeakIndex = -1;
    int maxPeakIndex = -1;
    std::chrono::system_clock::time_point peakTimes;
    float peakAmplitude;
};
