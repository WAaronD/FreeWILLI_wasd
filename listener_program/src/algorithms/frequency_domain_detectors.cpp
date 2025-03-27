#include "frequency_domain_detectors.h"

#include "../pch.h"

AverageMagnitudeDetector::AverageMagnitudeDetector(float threshold) : detectionThreshold(threshold) {}

bool AverageMagnitudeDetector::detect(const Eigen::VectorXcf& frequencyDomainData) const
{
    // Compute the average amplitude of the frequency-domain data
    float averageAmplitude = frequencyDomainData.array().abs().sum() / frequencyDomainData.size();

    return averageAmplitude >= detectionThreshold;
}

NoFrequencyDomainDetector::NoFrequencyDomainDetector() {}

bool NoFrequencyDomainDetector::detect(const Eigen::VectorXcf& /*frequencyDomainData*/) const { return true; }