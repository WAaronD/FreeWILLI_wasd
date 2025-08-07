#pragma once
#include "../pch.h"
#include "time_domain_filters.h"

class ITimeDomainFiltersFactory
{
   public:
    static std::unique_ptr<ITimeDomainFilter> create(
        const std::string& timeDomainDetector, const std::string& filterWeightsPath,
        Eigen::MatrixXf& channelData,  // pass by ref, so it can be resized if needed
        int numChannels)
    {
        if (timeDomainDetector == "FIRFilter")
        {
            return std::make_unique<FIRFilter>(filterWeightsPath, channelData);
        }
        else
        {
            throw std::invalid_argument("Unknown Time Domain Filter type: " + timeDomainDetector);
        }
    }
};