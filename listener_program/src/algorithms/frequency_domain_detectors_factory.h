#pragma once
#include "../pch.h"
#include "frequency_domain_detectors.h"
class IFrequencyDomainDetectorFactory
{
   public:
    static std::unique_ptr<IFrequencyDomainDetector> create(
        const std::string& frequencyDomainDetector, float energyDetectionThreshold)
    {
        if (frequencyDomainDetector == "AverageEnergy")
        {
            return std::make_unique<AverageMagnitudeDetector>(energyDetectionThreshold);
        }
        else if (frequencyDomainDetector == "None")
        {
            return std::make_unique<NoFrequencyDomainDetector>();
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + frequencyDomainDetector);
        }
    }
};