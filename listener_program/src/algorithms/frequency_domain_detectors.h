#pragma once
#include "../pch.h"

class IFrequencyDomainDetector
{
   public:
    virtual ~IFrequencyDomainDetector() = default;

    virtual bool detect(const Eigen::VectorXcf& frequencyDomainData) const = 0;
};

class AverageMagnitudeDetector : public IFrequencyDomainDetector
{
   private:
    float detectionThreshold;

   public:
    explicit AverageMagnitudeDetector(float threshold);

    bool detect(const Eigen::VectorXcf& frequencyDomainData) const override;
};

class NoFrequencyDomainDetector : public IFrequencyDomainDetector
{
   private:
    float detectionThreshold;

   public:
    explicit NoFrequencyDomainDetector();

    bool detect(const Eigen::VectorXcf& frequencyDomainData) const override;
};
