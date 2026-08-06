#pragma once
#include "../../pch.h"

class IFrequencyDomainDetector
{
   public:
    virtual ~IFrequencyDomainDetector() = default;

    virtual bool detect(const Eigen::VectorXcf& frequencyDomainData) = 0;
};

class AverageMagnitudeDetector : public IFrequencyDomainDetector
{
   public:
    explicit AverageMagnitudeDetector(float threshold);

    bool detect(const Eigen::VectorXcf& frequencyDomainData) override;

   private:
    float detectionThreshold;
};

class HampelBandEnergyDetector : public IFrequencyDomainDetector
{
   public:
    // bandStart/bandEnd are inclusive FFT-bin indices. Use -1/-1 for full band.
    HampelBandEnergyDetector(int windowSize = 31, float k = 2.0f, int bandStart = -1, int bandEnd = -1);

    bool detect(const Eigen::VectorXcf& X) override;

    // Optional runtime controls
    void setBand(int start, int end);
    void setK(float k);
    void setWindowSize(int w);

    // Telemetry
    float lastEnergy() const { return mLastEnergy; }
    float lastMedian() const { return mLastMedian; }
    float lastScaledMAD() const { return mLastScaledMAD; }
    float lastThreshold() const { return mLastThreshold; }

   private:
    static float medianInPlace(std::vector<float>& v);

    // State
    std::deque<float> mHistory;
    std::vector<float> mWork;

    int mWindowSize;
    float mK;
    int mBandStart;
    int mBandEnd;

    // Telemetry values
    float mLastEnergy = 0.f;
    float mLastMedian = 0.f;
    float mLastScaledMAD = 0.f;
    float mLastThreshold = 0.f;
};

class RuCCUSFDetector : public IFrequencyDomainDetector // Made with love by Claude
{
   public:
    RuCCUSFDetector(float f1, float f2, float srMin, float srMax, float sampleRate = 100000.f);
    bool detect(const Eigen::VectorXcf& X) override;
    float getLastSpectrumRatio() const { return mLastSr; }  // New!
   private:
    float mSrMin;
    float mSrMax;
    int mBin1;
    int mBin2;
    float mLastSr = 0.0f; // New!
};

struct FPeakLocationBand
{
    std::string label;
    float peakFreqMin;
    float peakFreqMax;
    float centerFreqMin;
    float centerFreqMax;
};

class FPeakLocationDetector : public IFrequencyDomainDetector
{
   public:
    FPeakLocationDetector(std::vector<FPeakLocationBand> bands, float sampleRate = 100000.f);
    bool detect(const Eigen::VectorXcf& X) override;

    float getLastPeakFrequency() const { return mLastPeakFreq; }
    float getLastCenterFrequency() const { return mLastCenterFreq; }
    int getLastMatchIndex() const { return mLastMatchIndex; }       // -1 if no match
    const std::string& getLastMatchLabel() const { return mLastMatchLabel; }

   private:
    std::vector<FPeakLocationBand> mBands;
    float mSampleRate;

    float mLastPeakFreq = 0.f;
    float mLastCenterFreq = 0.f;
    int mLastMatchIndex = -1;
    std::string mLastMatchLabel;
};