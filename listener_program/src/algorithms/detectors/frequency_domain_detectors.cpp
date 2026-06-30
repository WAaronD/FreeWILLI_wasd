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

RuCCUSFDetector::RuCCUSFDetector(float f1, float f2, float srMin, float srMax, float sampleRate) // Made with love by Claude
    : mSrMin(srMin), mSrMax(srMax)
{
    const int N = 992;  // FFT size

    auto toBin = [&](float freqHz) -> int {
        int bin = static_cast<int>(std::round(freqHz * N / sampleRate));
        if (bin < 0) bin = 0;
        if (bin >= N) bin = N - 1;
        return bin;
    };

    mBin1 = toBin(f1);
    mBin2 = toBin(f2);
}

bool RuCCUSFDetector::detect(const Eigen::VectorXcf& X) // Made with love by Claude
{
    if (X.size() <= 0) return false;

    const float amp1 = std::abs(X(mBin1));
    const float amp2 = std::abs(X(mBin2));

    if (amp1 <= 0.f) return false;

    const float sr = std::log10(amp2 / amp1);
    return (sr >= mSrMin) && (sr <= mSrMax);
}