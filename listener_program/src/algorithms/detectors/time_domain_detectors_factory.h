#pragma once
#include "../../pch.h"
#include "time_domain_detectors.h"

class ITimeDomainDetectorFactory
{
   public:
    static std::unique_ptr<ITimeDomainDetector> create(const nlohmann::json& params)
    {
        const std::string& detector = params.at("detector").get<std::string>();
        if (detector == "PeakAmplitude")
        {
            return std::make_unique<PeakAmplitudeDetector>(params.at("threshold").get<float>());
        }
        else if (detector == "RuCCUS")
        {
            return std::make_unique<RuCCUSDetector>(
                params.at("threshold").get<float>(),
                static_cast<int>(params.at("ocMin")),
                static_cast<int>(params.at("ocMax")));
        }
        else if (detector == "PeakLocation")
        {
            return std::make_unique<PeakLocationDetector>(params.at("threshold").get<float>());
        }
        else if (detector == "CFARDetector")
        {
            return std::make_unique<CFARPeakDetector>(
                static_cast<int>(params.at("numGuard")), static_cast<int>(params.at("numTrain")), params.at("pfa"));
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + detector);
        }
    }
};