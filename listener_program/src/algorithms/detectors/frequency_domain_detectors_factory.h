#pragma once
#include "../../pch.h"
#include "frequency_domain_detectors.h"
class IFrequencyDomainDetectorFactory
{
   public:
    static std::unique_ptr<IFrequencyDomainDetector> create(const nlohmann::json& params)
    {
        const std::string& detector = params.at("detector").get<std::string>();
        if (detector == "AverageEnergy")
        {
            return std::make_unique<AverageMagnitudeDetector>(params.at("threshold").get<float>());
        }
        if (detector == "HampelBandEnergyDetector")
        {
            return std::make_unique<HampelBandEnergyDetector>();
        }
        if (detector == "RuCCUSFDetector")
        {
            return std::make_unique<RuCCUSFDetector>(
                params.at("f1").get<float>(),
                params.at("f2").get<float>(),
                params.at("srMin").get<float>(),
                params.at("srMax").get<float>()
            );
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + detector);
        }
    }
};

