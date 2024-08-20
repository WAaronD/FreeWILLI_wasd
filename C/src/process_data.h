/*

This file contains all function prototypes for process_data.cpp

*/

#pragma once

#include "custom_types.h"

using TimePoint = std::chrono::system_clock::time_point;


void FrequencyDomainFIRFiltering(
    const Eigen::MatrixXf& channelData,           // Zero-padded time-domain data
    const Eigen::VectorXcf& filterFreq,           // Frequency domain filter (FIR taps in freq domain)
    fftwf_plan& FFTPlan,                           // FFT plan
    fftwf_plan& inverseFFTPlan,                    // Inverse FFT plan
    Eigen::MatrixXcf& savedFFTs,                  // Output of FFT transformed time-domain data
    Eigen::MatrixXf& invFFT);                      // Output of inverse FFT transformed data

void ConvertData(std::vector<float>& dataSegment,std::span<uint8_t> dataBytes,unsigned int& DATA_SIZE, unsigned int& HEAD_SIZE);

void GenerateTimestamps(std::vector<TimePoint>& dataTimes, std::span<uint8_t> dataBytes,unsigned int MICRO_INCR, bool& previousTimeSet, std::chrono::time_point<std::chrono::system_clock>& previousTime, std::string& detectionOutputFile, std::string& tdoaOutputFile, std::string& doaOutputFile);

DetectionResult ThresholdDetect(const Eigen::VectorXf& data, const std::span<TimePoint> times, const double& threshold, const unsigned int& SAMPLE_RATE);
void ProcessSegmentInterleaved(std::span<float> data, Eigen::MatrixXf& channelData, unsigned int NUM_CHAN);
