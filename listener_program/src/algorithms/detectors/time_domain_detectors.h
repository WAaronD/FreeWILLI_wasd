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

class AbsPeakAmplitudeDetector : public ITimeDomainDetector
{
   private:
    float detectionThreshold;
    float peakAmplitude;

   public:
    explicit AbsPeakAmplitudeDetector(float threshold);

    bool detect(const Eigen::VectorXf& timeDomainData) override;
    float getLastDetection() const override;
};

class RuCCUSDetector : public ITimeDomainDetector
{
   public:
    struct Params
    {
        float threshdet = 0.0f; // threshold detection value
        int minExcursions = 10; // minimum excursion count
        int maxExcursions = 25; // maximum excursion count
    };

    explicit RuCCUSDetector(float threshdet, int minExcursions = 10, int maxExcursions = 25);
    explicit RuCCUSDetector(const Params& p);

    bool detect(const Eigen::VectorXf& timeDomainData) override;

    float getLastDetection() const override;
    float getLastExcursionCount() const { return mLastExcursions; }

   private:
    void init_(const Params& p);

    float mThreshdet;
    int mMinExcursions;
    int mMaxExcursions;
    float mLastAmplitude; // stores actual peak value
    float mLastExcursions = 0.0f;
};

class PeakLocationDetector : public ITimeDomainDetector
{
   public:
    explicit PeakLocationDetector(float threshold);
    bool detect(const Eigen::VectorXf& timeDomainData) override;
    float getLastDetection() const override;

   private:
    float mThreshold;
    float mLastAmplitude;   // stores actual peak value
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

struct DurationBand
{
    std::string label;
    float durationMin; // microseconds
    float durationMax; // microseconds
};

class SignalDurationDetector : public ITimeDomainDetector
{
   public:
    SignalDurationDetector(std::vector<DurationBand> bands, float threshold, int edgeGuard = 30, float sampleRate = 100000.f);

    bool detect(const Eigen::VectorXf& timeDomainData) override;
    float getLastDetection() const override;    // returns duration in samples
    float getLastPeakAmplitude() const { return mLastPeakAmplitude; }

    const std::vector<bool>& getLastMatches() const { return mLastMatches; }

   private:
    std::vector<DurationBand> mBands;
    float mThreshold;
    int mEdgeGuard;
    float mSampleRate;

    float mLastDuration = 0.0f;
    float mLastPeakAmplitude = 0.0f;
    std::vector<bool> mLastMatches;
};