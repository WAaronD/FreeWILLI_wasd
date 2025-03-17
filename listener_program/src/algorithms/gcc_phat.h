#pragma once
#include "../pch.h"

/**
 * @brief Implements the Generalized Cross-Correlation with Phase Transform (GCC-PHAT) algorithm.
 *
 * This class estimates the Time Difference of Arrival (TDOA) between multiple microphone signals
 * by computing the cross-correlation of their frequency-domain representations. It applies PHAT
 * weighting to enhance accuracy, performs an inverse FFT to obtain the time-domain correlation,
 * and extracts the TDOA from the peak location.
 **/
class GCC_PHAT
{
   public:
    GCC_PHAT(int paddedLength, int spectraLength, int numChannels, int sampleRate);
    ~GCC_PHAT();

    std::tuple<Eigen::VectorXf, Eigen::VectorXf> process(const Eigen::MatrixXcf& savedFfts);

   private:
    void calculateNormalizedCrossSpectra(const Eigen::VectorXcf& s1, const Eigen::VectorXcf& s2);
    std::tuple<float, float> estimateTdoaAndPeak();

    int mPaddedLength;
    int mNumChannels;
    int mSampleRate;
    int mNumTdoas;
    int mMaxShift;

    Eigen::VectorXcf mNormalizedCrossSpectra;
    Eigen::VectorXf mCrossCorrBuffer;

    fftwf_plan mInverseFftPlan;
};