
#pragma once

#include "pch.h"

void WriteArray(const std::span<Eigen::VectorXf> array, const std::span<TimePoint> timestamps, const std::string &filename);

using TimePoint = std::chrono::system_clock::time_point;


class BufferWriter
{
private:
    void ClearBuffers(std::vector<Eigen::VectorXf>& buffer,std::vector<std::chrono::system_clock::time_point>& peakTimesBuffer)
    {
        buffer.clear();
        peakTimesBuffer.clear();
        _lastFlushTime = std::chrono::steady_clock::now();
    }

public:
    std::chrono::milliseconds _flushInterval;
    size_t _bufferSizeThreshold; // Adjust as needed
    std::chrono::time_point<std::chrono::steady_clock> _lastFlushTime;

    void write(std::vector<Eigen::VectorXf>& buffer,std::vector<std::chrono::system_clock::time_point>& peakTimesBuffer, const std::string& outputFile)
    {

        auto beforeW3 = std::chrono::steady_clock::now();
        WriteArray(buffer, peakTimesBuffer, outputFile);
        auto afterW3 = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationW3 = afterW3 - beforeW3;

        _lastFlushTime = std::chrono::steady_clock::now();
        ClearBuffers(buffer, peakTimesBuffer);
    }
};