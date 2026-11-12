#pragma once
#include "../pch.h"
#include "time_domain_filters.h"

class ITimeDomainFiltersFactory
{
   public:
    static std::unique_ptr<ITimeDomainFilter> create(
        const std::string& timeDomainDetector, const std::string& filterWeightsPath,
        Eigen::MatrixXf& channelData,  // pass by ref, so it can be resized if needed
        int numChannels, bool Ch1Only = false)
    {
        if (timeDomainDetector == "FIRFilter")
        {
            return std::make_unique<FIRFilter>(filterWeightsPath, channelData, Ch1Only);
        }
        else
        {
            throw std::invalid_argument("Unknown Time Domain Filter type: " + timeDomainDetector);
        }
    }
};