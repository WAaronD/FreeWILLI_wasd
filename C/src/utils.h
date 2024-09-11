/*

This file contains all function prototypes for utils.cpp

*/
#pragma once

#include "custom_types.h"

// Type aliases
using TimePoint = std::chrono::system_clock::time_point;

// File I/O functions
auto ReadFIRFilterFile(const char* fileName) -> std::vector<float>;
//bool ProcessFile(ExperimentConfig &exp, const std::string &fileName);
void InitiateOutputFile(std::string &outputFile, std::tm &timeStruct, int64_t microSec, std::string &feature, int NUM_CHAN);
void WritePulseAmplitudes(std::span<float> click_amps, std::span<TimePoint> timestamps, const std::string &filename);
void WriteArray(const std::span<Eigen::VectorXf> array, const std::span<TimePoint> timestamps, const std::string &filename);
void WriteDataToCerr(std::span<TimePoint> dataTimes, std::vector<std::vector<uint8_t>> dataBytesSaved);

// Data handling functions
auto LoadHydrophonePositions(const char* filename) -> Eigen::MatrixXd;
void ClearQueue(std::queue<std::vector<uint8_t>> &q);
bool WithProbability(double probability);

// Debug and print functions
void PrintTimes(const std::span<TimePoint> timestamps);
void PrintFirstFiveValues(const Eigen::MatrixXf &savedFFTs, const Eigen::MatrixXf &invFFT);
void PrintMode();

// Session and experiment management
void RestartListener(SocketManager &sess);
void InitializeSession(SocketManager& sess, ExperimentRuntime& expRuntime, char* argv[]);
//bool ConfigureExperiment(ExperimentConfig& expConfig, int firmwareVersion);

class BufferWriter
{
private:
    void ClearBuffers(Session &sess)
    {
        sess.Buffer.clear();
        sess.peakTimesBuffer.clear();
        _lastFlushTime = std::chrono::steady_clock::now();
    }

public:
    std::chrono::milliseconds _flushInterval;
    size_t _bufferSizeThreshold; // Adjust as needed
    std::chrono::time_point<std::chrono::steady_clock> _lastFlushTime;

    void write(Session &sess, ExperimentRuntime &expRuntime)
    {

        auto beforeW3 = std::chrono::steady_clock::now();
        WriteArray(sess.Buffer, sess.peakTimesBuffer, expRuntime.detectionOutputFile);
        auto afterW3 = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW3 = afterW3 - beforeW3;

        _lastFlushTime = std::chrono::steady_clock::now();
        ClearBuffers(sess);
    }
};