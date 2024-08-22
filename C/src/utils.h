/*

This file contains all function prototypes for utils.cpp

*/

#pragma once

#include "custom_types.h"

using TimePoint = std::chrono::system_clock::time_point;

std::vector<float> ReadFIRFilterFile(const std::string &fileName);
bool ProcessFile(Experiment &exp, const std::string &fileName);
void InitiateOutputFile(std::string &outputFile, std::tm &timeStruct, int64_t microSec, std::string &feature, int NUM_CHAN);
void PrintTimes(const std::span<TimePoint> timestamps);
Eigen::MatrixXd LoadHydrophonePositions(const std::string &filename);
void RestartListener(Session &sess);
void ClearQueue(std::queue<std::vector<uint8_t>> &q);
void printFirstFiveValues(const Eigen::MatrixXf &savedFFTs, const Eigen::MatrixXf &invFFT);
bool WithProbability(double probability);
void WritePulseAmplitudes(std::span<float> click_amps, std::span<TimePoint> timestamps, const std::string &filename);
void WriteArray(const std::span<Eigen::VectorXf> array, const std::span<TimePoint> timestamps, const std::string &filename);
void WriteDataToCerr(std::span<TimePoint> dataTimes, std::vector<std::vector<uint8_t>> dataBytesSaved);

class BufferWriter
{
private:
    void ClearBuffers(Session &sess)
    {
        sess.Buffer.clear();
    }

public:
    std::chrono::milliseconds _flushInterval;
    size_t _bufferSizeThreshold; // Adjust as needed
    std::chrono::time_point<std::chrono::steady_clock> _lastFlushTime;

    void write(Session &sess, Experiment &exp)
    {

        auto beforeW3 = std::chrono::steady_clock::now();
        WriteArray(sess.Buffer, sess.peakTimesBuffer, exp.detectionOutputFile);
        auto afterW3 = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW3 = afterW3 - beforeW3;

        _lastFlushTime = std::chrono::steady_clock::now();
        ClearBuffers(sess);
    }
};