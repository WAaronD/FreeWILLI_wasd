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
        if (detector == "FPeakLocationDetector")
        {
            std::vector<FPeakLocationBand> bands;
            for (const auto& b : params.at("bands"))
            {
                bands.push_back(FPeakLocationBand{
                    b.value("label", std::string("")),
                    b.at("peakFreqMin").get<float>(),
                    b.at("peakFreqMax").get<float>(),
                    b.at("centerFreqMin").get<float>(),
                    b.at("centerFreqMax").get<float>()
                });
            }
            return std::make_unique<FPeakLocationDetector>(
                std::move(bands),
                params.value("sampleRate", 100000.f)
            );
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + detector);
        }
    }
};

