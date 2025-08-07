#pragma once
#include "../../pch.h"
#include "time_domain_detectors.h"

class ITimeDomainDetectorFactory
{
   public:
    static std::unique_ptr<ITimeDomainDetector> create(const std::string& timeDomainDetector, float timeDomainThreshold)
    {
        if (timeDomainDetector == "PeakAmplitude")
        {
            return std::make_unique<PeakAmplitudeDetector>(timeDomainThreshold);
        }
        else if (timeDomainDetector == "RuCCUS")
        {
            return std::make_unique<RuCCUSDetector>(timeDomainThreshold);
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + timeDomainDetector);
        }
    }
};