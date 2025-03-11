#include "time_domain_detectors.h"

#include "../pch.h"

PeakAmplitudeDetector::PeakAmplitudeDetector(float threshold) : detectionThreshold(threshold), peakAmplitude(0) {}

bool PeakAmplitudeDetector::detect(const Eigen::VectorXf& timeDomainData)
{
    int peakIndex = 0;

    peakAmplitude = timeDomainData.maxCoeff(&peakIndex);

    return peakAmplitude >= detectionThreshold;
}

float PeakAmplitudeDetector::getLastDetection() const { return peakAmplitude; }