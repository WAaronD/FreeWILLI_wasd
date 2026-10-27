#include "frequency_domain_detectors.h"

#include "../pch.h"

AverageMagnitudeDetector::AverageMagnitudeDetector(float threshold) : detectionThreshold(threshold) {}

bool AverageMagnitudeDetector::detect(const Eigen::VectorXcf& frequencyDomainData)
{
    // Compute the average amplitude of the frequency-domain data
    float averageAmplitude = frequencyDomainData.array().abs().sum() / frequencyDomainData.size();

    return averageAmplitude >= detectionThreshold;
}

HampelBandEnergyDetector::HampelBandEnergyDetector(int windowSize, float k, int bandStart, int bandEnd)
    : mWindowSize(windowSize), mK(k), mBandStart(bandStart), mBandEnd(bandEnd)
{
    if (mWindowSize < 5) mWindowSize = 5;
    if ((mWindowSize & 1) == 0) ++mWindowSize;  // prefer odd
    if (mK <= 0.f) mK = 3.5f;
    mHistory.clear();
    mWork.reserve(static_cast<size_t>(mWindowSize));
}

bool HampelBandEnergyDetector::detect(const Eigen::VectorXcf& X)
{
    const int N = static_cast<int>(X.size());
    if (N <= 0) return false;

    const int b0 = (mBandStart >= 0 && mBandStart < N) ? mBandStart : 0;
    const int b1 = (mBandEnd >= 0 && mBandEnd < N && mBandEnd >= b0) ? mBandEnd : (N - 1);

    // 1) Band energy
    // float En = 0.f;
    // for (int k = b0; k <= b1; ++k) {
    // En += std::norm(X[k]);
    //}
    float En = X.array().abs().sum() / X.size();

    // 2) Update history buffer
    mHistory.push_back(En);
    if (static_cast<int>(mHistory.size()) < mWindowSize) return false;
    if (static_cast<int>(mHistory.size()) > mWindowSize) mHistory.pop_front();

    // 3) Median & MAD
    mWork.assign(mHistory.begin(), mHistory.end());
    const float m = medianInPlace(mWork);

    for (float& v : mWork) v = std::fabs(v - m);
    const float mad = medianInPlace(mWork);
    const float scaledMad = 1.4826f * mad;

    // 4) Threshold
    float thr;
    if (scaledMad == 0.f)
        thr = m;
    else
        thr = m + mK * scaledMad;

    mLastEnergy = En;
    mLastMedian = m;
    mLastScaledMAD = scaledMad;
    mLastThreshold = thr;
    if (En > thr)
    {
        std::cout << "Freq Thresh: " << thr << std::endl;
    }

    return (En > thr);
    // return (En > 100);
}

void HampelBandEnergyDetector::setBand(int start, int end)
{
    mBandStart = start;
    mBandEnd = end;
}

void HampelBandEnergyDetector::setK(float k)
{
    if (k > 0.f) mK = k;
}

void HampelBandEnergyDetector::setWindowSize(int w)
{
    if (w < 5) w = 5;
    if ((w & 1) == 0) ++w;
    mWindowSize = w;
    mHistory.clear();  // reset baseline
}

float HampelBandEnergyDetector::medianInPlace(std::vector<float>& v)
{
    const size_t n = v.size();
    if (n == 0) return 0.f;
    const size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float med = v[mid];
    if ((n & 1U) == 0U)
    {
        const float upper = med;
        std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
        const float lower = v[mid - 1];
        med = 0.5f * (lower + upper);
    }
    return med;
}

RuCCUSFDetector::RuCCUSFDetector(float f1, float f2, float srMin, float srMax, float sampleRate)
    : mSrMin(srMin), mSrMax(srMax) // srMin and srMax are copied to member variables for later use by the detect() method
{
    const int N = 992;  // FFT size

    // Convert frequencies f1 and f2 to FFT bin indices
    auto toBin = [&](float freqHz) -> int { // lambda function
        int bin = static_cast<int>(std::round(freqHz * N / sampleRate)); // Convert frequency to bin index
        if (bin < 0) bin = 0; // Clamp to 0 if below range
        if (bin >= N) bin = N - 1; // Clamp to N-1 if above range 
        return bin;
    };

    mBin1 = toBin(f1); // Convert f1 to FFT bin index
    mBin2 = toBin(f2); // Convert f2 to FFT bin index
}

bool RuCCUSFDetector::detect(const Eigen::VectorXcf& X)
{
    if (X.size() <= 0) return false;

    const float amp1 = std::abs(X(mBin1)); // abs is required because X is complex-valued
    const float amp2 = std::abs(X(mBin2));

    if (amp1 <= 0.f) return false; // Avoid dividing by zero

    const float sr = std::log10(amp2 / amp1);
    mLastSr = sr; // New!

    return (sr >= mSrMin) && (sr <= mSrMax);
}

FPeakLocationDetector::FPeakLocationDetector(std::vector<FPeakLocationBand> bands, float sampleRate)
    : mBands(std::move(bands)), mSampleRate(sampleRate) {}

bool FPeakLocationDetector::detect(const Eigen::VectorXcf& X)
{
    const int N = static_cast<int>(X.size());
    if (N <= 0) return false;

    int peakBin = 0;
    float peakMagSq = 0.f;
    float magSqSum = 0.f;
    float weightedBinSum = 0.f;

    const int N_full = (N - 1) * 2;  // reconstruct full FFT size = 992
    for (int k = 0; k < N; ++k)
    {
        const float magSq = std::norm(X(k));  // re*re + im*im, no sqrt
        magSqSum += magSq;
        weightedBinSum += k * magSq;
        if (magSq > peakMagSq)
        {
            peakMagSq = magSq;
            peakBin = k;
        }
    }

    mLastPeakFreq = peakBin * mSampleRate / N_full;
    mLastCenterFreq = (magSqSum > 0.f) ? (weightedBinSum / magSqSum) * mSampleRate / N_full : 0.f;

    // Check ALL bands, mark each as matched or not, independently for each metric
    mLastPeakMatches.assign(mBands.size(), false);
    mLastCenterMatches.assign(mBands.size(), false);

    bool peakAnyMatch = false;
    bool centerAnyMatch = false;
    for (size_t i = 0; i < mBands.size(); ++i)
    {
        const auto& b = mBands[i];
        const bool peakOk = (mLastPeakFreq >= b.peakFreqMin) && (mLastPeakFreq <= b.peakFreqMax);
        const bool centerOk = (mLastCenterFreq >= b.centerFreqMin) && (mLastCenterFreq <= b.centerFreqMax);

        mLastPeakMatches[i] = peakOk;
        mLastCenterMatches[i] = centerOk;

        if (peakOk) peakAnyMatch = true;
        if (centerOk) centerAnyMatch = true;
    }

    // Toss the signal if EITHER vector is all-zero
    return peakAnyMatch && centerAnyMatch;
}

BandwidthDetector::BandwidthDetector(std::vector<BandwidthBand> bands, float sampleRate)
    : mBands(std::move(bands)), mSampleRate(sampleRate)
{
}

namespace {
    constexpr float kThreshRatio3dB = 0.5011872f;  // 10^(-3/10), linear power ratio
    constexpr float kThreshRatio10dB = 0.1f;       // 10^(-10/10)
}

float BandwidthDetector::computeBandwidthHz(
    const Eigen::VectorXcf& X, int peakBin, float thresh, int nFull) const
{
    const int N = static_cast<int>(X.size());

    int lo = peakBin;
    while (lo > 0 && std::norm(X(lo)) >= thresh) --lo;  // first bin below threshold (or bin 0)

    int hi = peakBin;
    while (hi < N - 1 && std::norm(X(hi)) >= thresh) ++hi;  // first bin below threshold (or last bin)

    return static_cast<float>(hi - lo) * mSampleRate / nFull;
}

bool BandwidthDetector::detect(const Eigen::VectorXcf& X)
{
    mLastBw3dB = 0.f;
    mLastBw10dB = 0.f;
    mLastBw3dBMatches.assign(mBands.size(), false);
    mLastBw10dBMatches.assign(mBands.size(), false);

    const int N = static_cast<int>(X.size());
    if (N <= 0) return false;

    const int nFull = (N - 1) * 2;  // full FFT length from one-sided size

    // Peak search (first maximum, same as MATLAB's max)
    int peakBin = 0;
    float peakMagSq = 0.f;
    for (int k = 0; k < N; ++k)
    {
        const float m = std::norm(X(k));
        if (m > peakMagSq)
        {
            peakMagSq = m;
            peakBin = k;
        }
    }
    if (peakMagSq <= 0.f) return false;

    mLastBw3dB = computeBandwidthHz(X, peakBin, peakMagSq * kThreshRatio3dB, nFull);
    mLastBw10dB = computeBandwidthHz(X, peakBin, peakMagSq * kThreshRatio10dB, nFull);

    bool bw3AnyMatch = false;
    bool bw10AnyMatch = false;
    for (size_t i = 0; i < mBands.size(); ++i)
    {
        const auto& b = mBands[i];
        const bool bw3Ok = (mLastBw3dB >= b.bw3dBMin) && (mLastBw3dB <= b.bw3dBMax);
        const bool bw10Ok = (mLastBw10dB >= b.bw10dBMin) && (mLastBw10dB <= b.bw10dBMax);

        mLastBw3dBMatches[i] = bw3Ok;
        mLastBw10dBMatches[i] = bw10Ok;

        if (bw3Ok) bw3AnyMatch = true;
        if (bw10Ok) bw10AnyMatch = true;
    }

    return bw3AnyMatch && bw10AnyMatch;
}