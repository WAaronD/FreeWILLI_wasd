#include "time_domain_detectors.h"

#include "../pch.h"

PeakAmplitudeDetector::PeakAmplitudeDetector(float threshold) : detectionThreshold(threshold), peakAmplitude(0) {}

bool PeakAmplitudeDetector::detect(const Eigen::VectorXf& timeDomainData)
{
    int peakIndex = 0;

    peakAmplitude = timeDomainData.maxCoeff(&peakIndex);

    return peakAmplitude >= detectionThreshold;
}

float PeakAmplitudeDetector::getLastDetection() const { return peakAmplitude; }

// RuCCUSDetector::RuCCUSDetector(float threshdet) : RuCCUSDetector(Params{threshdet, 10, 25}) {} // Old constructor, now replaced by the new one below
RuCCUSDetector::RuCCUSDetector(float threshdet, int minExcursions, int maxExcursions) // New constructor that takes individual parameters
{
    Params p;
    p.threshdet = threshdet;
    p.minExcursions = minExcursions;
    p.maxExcursions = maxExcursions;
    init_(p);
}

RuCCUSDetector::RuCCUSDetector(const Params& p) // New constructor that takes a Params struct
{
    init_(p);
}

void RuCCUSDetector::init_(const Params& p) // New initialization function that takes a Params struct
{
    mThreshdet = p.threshdet;
    mMinExcursions = p.minExcursions;
    mMaxExcursions = p.maxExcursions;
    mLastAmplitude = 0.0f;
}

bool RuCCUSDetector::detect(const Eigen::VectorXf& data)
{
    // 1) Skip if no sample exceeds threshold
    int countAbove = (data.array().abs() > mThreshdet).template cast<int>().sum();
    if (countAbove == 0)
    {
        mLastAmplitude = 0.0f;
        return false;
    }

    // 2) Find index of absolute max
    Eigen::Index idxMax;
    float maxVal = data.array().abs().maxCoeff(&idxMax);

    // 3) Skip if too close to edges
    if (idxMax < 31 || idxMax > data.size() - 80)
    {
        mLastAmplitude = 0.0f;
        return false;
    }

    // 4) Extract 60-sample windows
    Eigen::VectorXf click = data.segment(idxMax - 30, 60);
    Eigen::VectorXf clickl = data.segment(idxMax - 31, 60);

    // 5) Compute zero-crossing threshold
    float zxThresh = 0.20f * maxVal;
    float minThresh = 0.20f * mThreshdet;
    if (zxThresh < minThresh) zxThresh = minThresh;

    // 6) Count up/down excursions
    int excUp = 0, excDown = 0;
    for (int i = 0; i < click.size(); ++i)
    {
        if (click(i) > zxThresh && clickl(i) < zxThresh) ++excUp;
        if (click(i) < -zxThresh && clickl(i) > -zxThresh) ++excDown;
    }
    int excursions = excUp + excDown;
    mLastExcursions = static_cast<float>(excursions); // New!

    // 7) Final decision
    bool result = (excursions >= mMinExcursions && excursions <= mMaxExcursions);
    mLastAmplitude = result ? maxVal : 0.0f; // Changed: store real amplitude, not 1.0f/0.0f
    return result;
}

// float RuCCUSDetector::getLastDetection() const { return mLastDetection; }
float RuCCUSDetector::getLastDetection() const { return mLastAmplitude; } // Changed: return real amplitude, not 1.0f/0.0f

PeakLocationDetector::PeakLocationDetector(float threshold) : mThreshold(threshold), mLastAmplitude(0.0f) {}

bool PeakLocationDetector::detect(const Eigen::VectorXf& data) // Made with love by Claude
{
    // 1) Find index and value of peak (max absolute value)
    Eigen::Index idxMax;
    float peakVal = data.array().abs().maxCoeff(&idxMax);

    // 2) Check peak amplitude against threshold A
    if (peakVal <= mThreshold)
    {
        mLastAmplitude = 0.0f;
        return false;
    }

    // 3) Check peak isn't too close to either edge
    if (idxMax < 30 || idxMax > data.size() - 80)
    {
        mLastAmplitude = 0.0f;
        return false;
    }

    mLastAmplitude = peakVal; // Store the actual peak value
    return true;
}

float PeakLocationDetector::getLastDetection() const { return mLastAmplitude; }

CFARPeakDetector::CFARPeakDetector(int numGuard, int numTrain, float pfa, bool useAbs, bool requireLocalMax)
{
    Params p;
    p.numGuard = numGuard;
    p.numTrain = numTrain;
    p.pfa = pfa;
    p.useAbs = useAbs;
    p.requireLocalMax = requireLocalMax;
    init_(p);
}

CFARPeakDetector::CFARPeakDetector(const Params& p) { init_(p); }

bool CFARPeakDetector::detect(const Eigen::VectorXf& timeDomainData)
{
    const int N = static_cast<int>(timeDomainData.size());
    mLastPeak = 0.0f;
    mLastIndex = -1;
    mLastThreshold = 0.0f;

    if (N <= 0) return false;

    // Optional absolute value
    Eigen::VectorXf x = mUseAbs ? timeDomainData.cwiseAbs() : timeDomainData;

    // Prefix sums for O(1) range sums
    Eigen::VectorXf pref(N + 1);
    pref[0] = 0.0f;
    for (int i = 0; i < N; ++i) pref[i + 1] = pref[i] + x[i];

    const int margin = mNumTrain + mNumGuard;
    const int iStart = margin;
    const int iEnd = N - margin - 1;
    if (iStart > iEnd) return false;  // vector too short for current CFAR window

    bool found = false;
    float bestVal = -std::numeric_limits<float>::infinity();
    int bestIdx = -1;
    float bestThr = 0.0f;

    for (int i = iStart; i <= iEnd; ++i)
    {
        // Training windows:
        // Left:  [i - G - T, i - G)
        // Right: [i + G + 1, i + G + T]
        const int leftStart = i - mNumGuard - mNumTrain;
        const int leftEnd = i - mNumGuard;  // exclusive in prefix math
        const int rightStart = i + mNumGuard + 1;
        const int rightEnd = i + mNumGuard + 1 + mNumTrain;

        if (leftStart < 0 || rightEnd > N) continue;  // safety

        const float leftSum = pref[leftEnd] - pref[leftStart];
        const float rightSum = pref[rightEnd] - pref[rightStart];
        const float noiseAvg = (leftSum + rightSum) / static_cast<float>(2 * mNumTrain);

        const float xi = x[i];
        const float thr = mAlpha * noiseAvg;

        if (xi > thr)
        {
            if (mRequireLocalMax)
            {
                const float xm1 = x[i - 1];
                const float xp1 = x[i + 1];
                if (!(xi >= xm1 && xi >= xp1)) continue;
            }
            if (xi > bestVal)
            {
                bestVal = xi;
                bestIdx = i;
                bestThr = thr;
                found = true;
            }
        }
    }

    if (found)
    {
        mLastPeak = bestVal;
        mLastIndex = bestIdx;
        mLastThreshold = bestThr;
    }
    return found;
}

float CFARPeakDetector::getLastDetection() const { return mLastPeak; }

int CFARPeakDetector::getLastIndex() const { return mLastIndex; }

float CFARPeakDetector::getLastThreshold() const { return mLastThreshold; }

void CFARPeakDetector::init_(const Params& p)
{
    if (p.numTrain <= 0) throw std::invalid_argument("CFAR: numTrain must be > 0");
    if (p.numGuard < 0) throw std::invalid_argument("CFAR: numGuard must be >= 0");
    if (!(p.pfa > 0.0f && p.pfa < 1.0f)) throw std::invalid_argument("CFAR: pfa must be in (0,1)");

    mNumGuard = p.numGuard;
    mNumTrain = p.numTrain;
    mPfa = p.pfa;
    mUseAbs = p.useAbs;
    mRequireLocalMax = p.requireLocalMax;

    const int nTrainTotal = 2 * mNumTrain;
    // CA-CFAR multiplier for exponential noise model:
    // alpha = N * (Pfa^(-1/N) - 1)
    mAlpha = static_cast<float>(nTrainTotal * (std::pow(mPfa, -1.0 / nTrainTotal) - 1.0));
}

SignalDurationDetector::SignalDurationDetector(std::vector<DurationBand> bands, float threshold, int edgeGuard)
    : mBands(std::move(bands)), mThreshold(threshold), mEdgeGuard(edgeGuard) {}

bool SignalDurationDetector::detect(const Eigen::VectorXf& data)
{
    mLastDuration = 0.0f;
    mLastMatchIndex = -1;
    mLastMatchLabel.clear();

    // 1) Gate on peak amplitude
    int peakIndex = 0;
    float peakVal = data.array().abs().maxCoeff(&peakIndex);
    if (peakVal <= mThreshold) return false;

    // 2) Cumulative energy (integrated squared pressure)
    Eigen::VectorXf energy = data.array().square();
    float totalEnergy = energy.sum();
    if (totalEnergy <= 0.0f) return false;

    float cum = 0.0f;
    int idx5 = -1, idx95 = -1;
    for (int i = 0; i < energy.size(); ++i)
    {
        cum += energy(i);
        if (idx5 < 0 && cum >= 0.05f * totalEnergy) idx5 = i;
        if (idx95 < 0 && cum >= 0.95f * totalEnergy)
        {
            idx95 = i;
            break;
        }
    }

    // 3) Reject if either edge of the 5-95% window is too close to the segment boundary
    if (idx5 < mEdgeGuard || idx95 > data.size() - 1 - mEdgeGuard) return false;

    mLastDuration = static_cast<float>(idx95 - idx5 + 1);

    // 4) Check bands in order; first match wins
    for (size_t i = 0; i < mBands.size(); ++i)
    {
        const auto& b = mBands[i];
        if (mLastDuration >= b.durationMin && mLastDuration <= b.durationMax)
        {
            mLastMatchIndex = static_cast<int>(i);
            mLastMatchLabel = b.label;
            break;
        }
    }

    return mLastMatchIndex >= 0;
}

float SignalDurationDetector::getLastDetection() const { return mLastDuration; }