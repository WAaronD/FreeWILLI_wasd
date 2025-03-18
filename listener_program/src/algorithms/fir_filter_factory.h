#pragma once
#include "../pch.h"
#include "fir_filter.h"

class IFrequencyDomainStrategyFactory
{
   public:
    static std::unique_ptr<IFrequencyDomainStrategy> create(
        const std::string& frequencyDomainStrategy, const std::string& filterWeightsPath,
        Eigen::MatrixXf& channelData,  // pass by ref, so it can be resized if needed
        int numChannels)
    {
        if (frequencyDomainStrategy == "None")
        {
            return std::make_unique<FrequencyDomainNoFilterStrategy>(channelData, numChannels);
        }
        else if (frequencyDomainStrategy == "Filter")
        {
            return std::make_unique<FrequencyDomainFilterStrategy>(filterWeightsPath, channelData, numChannels);
        }
        else
        {
            throw std::invalid_argument("Unknown frequency domain strategy: " + frequencyDomainStrategy);
        }
    }
};