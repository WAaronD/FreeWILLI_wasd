#pragma once
#include "../pch.h"
#include "frequency_domain_transforms.h"

class IFrequencyDomainTransformFactory
{
   public:
    static std::unique_ptr<IFrequencyDomainTransform> create(
        const std::string& frequencyDomainStrategy, const std::string& filterWeightsPath,
        Eigen::MatrixXf& channelData,  // pass by ref, so it can be resized if needed
        int numChannels, bool Ch1Only = false)
    {
        if (frequencyDomainStrategy == "None")
        {
            return std::make_unique<FrequencyDomainTransform>(channelData, numChannels, Ch1Only);
        }
        else if (frequencyDomainStrategy == "Filter")
        {
            return std::make_unique<FrequencyDomainFIRFilter>(filterWeightsPath, channelData, numChannels, Ch1Only);
        }
        else
        {
            throw std::invalid_argument("Unknown frequency domain strategy: " + frequencyDomainStrategy);
        }
    }
};