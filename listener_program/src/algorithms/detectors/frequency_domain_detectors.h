#pragma once
#include "../../pch.h"

class IFrequencyDomainDetector
{
   public:
    virtual ~IFrequencyDomainDetector() = default;

    virtual bool detect(const Eigen::VectorXcf& frequencyDomainData) const = 0;
};

class AverageMagnitudeDetector : public IFrequencyDomainDetector
{
   public:
    explicit AverageMagnitudeDetector(float threshold);

    bool detect(const Eigen::VectorXcf& frequencyDomainData) const override;

   private:
    float detectionThreshold;
};

class NoFrequencyDomainDetector : public IFrequencyDomainDetector
{
   public:
    explicit NoFrequencyDomainDetector();

    bool detect(const Eigen::VectorXcf& frequencyDomainData) const override;

   private:
    float detectionThreshold;
};
