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

RuCCUSDetector::RuCCUSDetector(float threshdet) : mThreshdet(threshdet), mLastDetection(0.0f) {}

bool RuCCUSDetector::detect(const Eigen::VectorXf& data)
{
    // 1) Skip if no sample exceeds threshold
    int countAbove = (data.array().abs() > mThreshdet).template cast<int>().sum();
    if (countAbove == 0)
    {
        mLastDetection = 0.0f;
        return false;
    }

    // 2) Find index of absolute max
    Eigen::Index idxMax;
    float maxVal = data.array().abs().maxCoeff(&idxMax);

    // 3) Skip if too close to edges
    if (idxMax < 30 || idxMax > data.size() - 30)
    {
        mLastDetection = 0.0f;
        return false;
    }

    // 4) Extract 60-sample windows
    Eigen::VectorXf click = data.segment(idxMax - 30, 60);
    Eigen::VectorXf clickl = data.segment(idxMax - 31, 60);

    // 5) Compute zero-crossing threshold
    float zxThresh = 0.20f * maxVal;
    float minThresh = 0.20f * mThreshdet;
    if (zxThresh < minThresh) zxThresh = minThresh;

    // 6) Count up/down excursions
    int excUp = 0, excDown = 0;
    for (int i = 0; i < click.size(); ++i)
    {
        if (click(i) > zxThresh && clickl(i) < zxThresh) ++excUp;
        if (click(i) < -zxThresh && clickl(i) > -zxThresh) ++excDown;
    }
    int excursions = excUp + excDown;

    // 7) Final decision
    bool result = (excursions >= 10);
    std::cout << "Ruccus result: " << result << " \n";
    mLastDetection = result ? 1.0f : 0.0f;
    return result;
}

float RuCCUSDetector::getLastDetection() const { return mLastDetection; }