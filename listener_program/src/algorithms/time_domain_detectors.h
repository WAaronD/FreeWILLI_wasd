#pragma once
#include "../pch.h"

class ITimeDomainDetector
{
   public:
    virtual ~ITimeDomainDetector() = default;
    virtual bool detect(const Eigen::VectorXf& timeDomainData) = 0;
    virtual float getLastDetection() const = 0;
};

class PeakAmplitudeDetector : public ITimeDomainDetector
{
   private:
    float detectionThreshold;
    float peakAmplitude;

   public:
    explicit PeakAmplitudeDetector(float threshold);

    bool detect(const Eigen::VectorXf& timeDomainData) override;
    float getLastDetection() const override;
};

class NoTimeDomainDetector : public ITimeDomainDetector
{
   private:
    float peakAmplitude;

   public:
    NoTimeDomainDetector() = default;

    bool detect(const Eigen::VectorXf& timeDomainData) override
    {
        int peakIndex = 0;

        peakAmplitude = timeDomainData.maxCoeff(&peakIndex);

        return true;
    }

    float getLastDetection() const override { return peakAmplitude; }
};

class ITimeDomainDetectorFactory
{
   public:
    static std::unique_ptr<ITimeDomainDetector> create(const std::string& timeDomainDetector, float timeDomainThreshold)
    {
        if (timeDomainDetector == "PeakAmplitude")
        {
            return std::make_unique<PeakAmplitudeDetector>(timeDomainThreshold);
        }
        else if (timeDomainDetector == "None")
        {
            return std::make_unique<NoTimeDomainDetector>();
        }
        else
        {
            throw std::invalid_argument("Unknown TimeDomainDetector type: " + timeDomainDetector);
        }
    }
};