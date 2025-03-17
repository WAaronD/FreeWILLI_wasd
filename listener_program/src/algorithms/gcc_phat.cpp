#include "gcc_phat.h"
GCC_PHAT::GCC_PHAT(int paddedLength, int spectraLength, int numChannels, int sampleRate)
    : mPaddedLength(paddedLength),
      mNumChannels(numChannels),
      mSampleRate(sampleRate),
      mNumTdoas(numChannels * (numChannels - 1) / 2),
      mMaxShift(paddedLength / 2),
      mNormalizedCrossSpectra(Eigen::VectorXcf::Zero(spectraLength)),
      mCrossCorrBuffer(Eigen::VectorXf::Zero(paddedLength))
{
    mInverseFftPlan = fftwf_plan_dft_c2r_1d(
        mPaddedLength, reinterpret_cast<fftwf_complex*>(mNormalizedCrossSpectra.data()), mCrossCorrBuffer.data(),
        FFTW_ESTIMATE);
}

GCC_PHAT::~GCC_PHAT() { fftwf_destroy_plan(mInverseFftPlan); }

/**
 * @brief Computes Time Difference of Arrival (TDOA) estimates and cross-correlation peaks using GCC-PHAT.
 *
 * This function processes the saved FFTs of input signals, computes the normalized cross-spectra
 * for each microphone pair, and applies an inverse FFT to estimate TDOA values and their corresponding
 * cross-correlation peak magnitudes.
 *
 * @param savedFfts A matrix of saved FFTs, where each column represents the FFT of a different microphone channel.
 *
 * @return A tuple containing:
 *         - Eigen::VectorXf: A vector of estimated TDOA values for each microphone pair.
 *         - Eigen::VectorXf: A vector of cross-correlation peak magnitudes corresponding to each TDOA.
 */
auto GCC_PHAT::process(const Eigen::MatrixXcf& savedFfts) -> std::tuple<Eigen::VectorXf, Eigen::VectorXf>
{
    Eigen::VectorXf tdoaEstimates(mNumTdoas);
    Eigen::VectorXf crossCorrPeaks(mNumTdoas);

    int pairIndex = 0;
    for (int ch1 = 0; ch1 < mNumChannels - 1; ++ch1)
    {
        for (int ch2 = ch1 + 1; ch2 < mNumChannels; ++ch2)
        {
            calculateNormalizedCrossSpectra(savedFfts.col(ch1), savedFfts.col(ch2));

            fftwf_execute(mInverseFftPlan);
            auto [tdoa, peak] = estimateTdoaAndPeak();
            tdoaEstimates(pairIndex) = tdoa;
            crossCorrPeaks(pairIndex) = peak;
            ++pairIndex;
        }
    }

    return {tdoaEstimates, crossCorrPeaks};
}

/**
 * @brief Computes the normalized cross-spectral density between two input signals.
 *
 * This function calculates the cross-spectrum of two FFT-transformed signals, normalizes it
 * using its magnitude, and stores the result in `mNormalizedCrossSpectra`. If the cross-spectrum
 * contains invalid values (NaN or infinity), an exception is thrown.
 *
 * @param[in] s1 The FFT-transformed signal from the first microphone/channel.
 * @param[in] s2 The FFT-transformed signal from the second microphone/channel.
 *
 * @throws std::runtime_error If the computed cross-spectrum contains NaN or infinite values.
 */
void GCC_PHAT::calculateNormalizedCrossSpectra(const Eigen::VectorXcf& s1, const Eigen::VectorXcf& s2)
{
    Eigen::VectorXcf crossSpectrum = s1.array() * s2.conjugate().array();
    Eigen::VectorXf magnitudes = crossSpectrum.cwiseAbs().unaryExpr([](float x) { return (x == 0.0f) ? 1.0f : x; });

    if (!magnitudes.allFinite())
    {
        throw std::runtime_error("Cross-spectrum contains invalid (inf/NaN) values.");
    }

    mNormalizedCrossSpectra.array() = crossSpectrum.array() / magnitudes.array();
}

/**
 * @brief Estimates the Time Difference of Arrival (TDOA) and cross-correlation peak value.
 *
 * This function rearranges the cross-correlation buffer to handle circular shifts properly,
 * finds the peak cross-correlation value, and determines the corresponding time shift.
 * The computed TDOA (Time Difference of Arrival) is obtained by normalizing the shift
 * based on the sample rate.
 *
 * @return A tuple containing:
 *         - `float` : The estimated TDOA value in seconds.
 *         - `float` : The peak value of the cross-correlation function, indicating signal similarity.
 */
auto GCC_PHAT::estimateTdoaAndPeak() -> std::tuple<float, float>
{
    Eigen::VectorXf rearranged(2 * mMaxShift);
    rearranged.head(mMaxShift) = mCrossCorrBuffer.tail(mMaxShift);
    rearranged.tail(mMaxShift) = mCrossCorrBuffer.head(mMaxShift);

    Eigen::Index peakIndex;
    float peakVal = rearranged.maxCoeff(&peakIndex);

    float shift = static_cast<float>(peakIndex) - mMaxShift;
    float tdoa = shift / static_cast<float>(mSampleRate);

    return {tdoa, peakVal};
}