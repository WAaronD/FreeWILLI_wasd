#pragma once
#include "../../pch.h"

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

class RuCCUSDetector : public ITimeDomainDetector
{
   public:
    explicit RuCCUSDetector(float threshdet);

    bool detect(const Eigen::VectorXf& timeDomainData) override;

    float getLastDetection() const override;

   private:
    float mThreshdet;
    float mLastDetection;  // 1.0f = click detected, 0.0f = no click
};

class CFARPeakDetector : public ITimeDomainDetector
{
   public:
    struct Params
    {
        int numGuard = 2;  // guard cells on EACH side
        int numTrain = 16;  // training cells on EACH side
        float pfa = 1e-2f;  // desired false alarm probability (0<pfa<1)
        bool useAbs = true;  // operate on |x[n]|
        bool requireLocalMax = true;  // only accept local maxima
    };

    // ctor matching factory usage
    explicit CFARPeakDetector(int numGuard, int numTrain, float pfa, bool useAbs = true, bool requireLocalMax = true);

    // convenience ctor
    explicit CFARPeakDetector(const Params& p);

    bool detect(const Eigen::VectorXf& timeDomainData) override;
    float getLastDetection() const override;

    // optional convenience getters
    int getLastIndex() const;
    float getLastThreshold() const;

   private:
    void init_(const Params& p);

    // configuration
    int mNumGuard = 2;
    int mNumTrain = 16;
    float mPfa = 1e-3f;
    bool mUseAbs = true;
    bool mRequireLocalMax = true;
    float mAlpha = 0.0f;  // CA-CFAR multiplier (from Pfa & training size)

    // last detection results
    float mLastPeak = 0.0f;
    int mLastIndex = -1;
    float mLastThreshold = 0.0f;
};