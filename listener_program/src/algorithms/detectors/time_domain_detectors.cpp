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

RuCCUSDetector::RuCCUSDetector(float threshdet) : m_threshdet(threshdet), m_lastDetection(0.0f) {}

bool RuCCUSDetector::detect(const Eigen::VectorXf& data)
{
    // 1) Skip if no sample exceeds threshold
    int countAbove = (data.array().abs() > m_threshdet).template cast<int>().sum();
    if (countAbove == 0)
    {
        m_lastDetection = 0.0f;
        return false;
    }

    // 2) Find index of absolute max
    Eigen::Index idx_max;
    float maxVal = data.array().abs().maxCoeff(&idx_max);

    // 3) Skip if too close to edges
    if (idx_max < 30 || idx_max > data.size() - 30)
    {
        m_lastDetection = 0.0f;
        return false;
    }

    // 4) Extract 60-sample windows
    Eigen::VectorXf click = data.segment(idx_max - 30, 60);
    Eigen::VectorXf clickl = data.segment(idx_max - 31, 60);

    // 5) Compute zero-crossing threshold
    float zx_thresh = 0.20f * maxVal;
    float min_thresh = 0.20f * m_threshdet;
    if (zx_thresh < min_thresh) zx_thresh = min_thresh;

    // 6) Count up/down excursions
    int exc_up = 0, exc_down = 0;
    for (int i = 0; i < click.size(); ++i)
    {
        if (click(i) > zx_thresh && clickl(i) < zx_thresh) ++exc_up;
        if (click(i) < -zx_thresh && clickl(i) > -zx_thresh) ++exc_down;
    }
    int excursions = exc_up + exc_down;

    // 7) Final decision
    bool result = (excursions >= 10);
    std::cout << "Ruccus result: " << result << " \n";
    m_lastDetection = result ? 1.0f : 0.0f;
    return result;
}

float RuCCUSDetector::getLastDetection() const { return m_lastDetection; }