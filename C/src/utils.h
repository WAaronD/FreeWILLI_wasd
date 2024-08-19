/*

This file contains all function prototypes for utils.cpp

*/

#pragma once

#include "custom_types.h"

using TimePoint = std::chrono::system_clock::time_point;

std::vector<float> ReadFIRFilterFile(const std::string& fileName);
bool ProcessFile(Experiment& exp, const std::string& fileName); 
void InitiateOutputFile(std::string& outputFile, std::tm& timeStruct, int64_t microSec, std::string& feature);
void PrintTimes(const std::span<TimePoint> timestamps);
std::vector<std::vector<float>> LoadHydrophonePositions(const std::string& filename);
void RestartListener(Session& sess);
void ClearQueue(std::queue<std::vector<uint8_t>>& q);
bool WithProbability(double probability);
void WritePulseAmplitudes(std::span<float> click_amps, std::span<TimePoint> timestamps, const std::string& filename);
void WriteArray(const std::span<Eigen::VectorXf> array, const std::span<TimePoint> timestamps, const std::string& filename);
void WriteDataToCerr(std::span<TimePoint> dataTimes, std::vector<std::vector<uint8_t>> dataBytesSaved);

class BufferWriter{
private:
    void ClearBuffers(Session& sess){
        sess.peakAmplitudeBuffer.clear();
        sess.peakTimesBuffer.clear();
        sess.resultMatrixBuffer.clear();
        sess.DOAsBuffer.clear();
    }
public:

    std::chrono::milliseconds _flushInterval;
    size_t _bufferSizeThreshold; // Adjust as needed
    std::chrono::time_point<std::chrono::steady_clock> _lastFlushTime;

    void write(Session& sess, Experiment& exp){
            
        auto beforeW = std::chrono::steady_clock::now();
        WritePulseAmplitudes(sess.peakAmplitudeBuffer, sess.peakTimesBuffer, exp.detectionOutputFile);
        auto afterW = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW = afterW - beforeW;
        std::cout << "Write: " << durationW.count() << std::endl;

        auto beforeW1 = std::chrono::steady_clock::now();
        WriteArray(sess.resultMatrixBuffer, sess.peakTimesBuffer, exp.tdoaOutputFile);
        auto afterW1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW1 = afterW1 - beforeW1;
        std::cout << "Write1: " << durationW1.count() << std::endl;

        auto beforeW2 = std::chrono::steady_clock::now();
        WriteArray(sess.DOAsBuffer, sess.peakTimesBuffer, exp.doaOutputFile);
        auto afterW2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW2 = afterW2 - beforeW2;
        std::cout << "Write:2 " << durationW2.count() << std::endl;
        
        _lastFlushTime = std::chrono::steady_clock::now();
        ClearBuffers(sess);
    }
};