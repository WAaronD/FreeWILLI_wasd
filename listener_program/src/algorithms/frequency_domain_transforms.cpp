#include "frequency_domain_transforms.h"

#include "filter_utility_functions.h"

FrequencyDomainFIRFilter::FrequencyDomainFIRFilter(
    const std::string& filterPath, Eigen::MatrixXf& channelData, int numChannels)
    : mNumChannels(numChannels)
{
    auto filterWeights = readFirFilterFile(filterPath);
    mPaddedLength = static_cast<int>(filterWeights.size() + channelData.cols() - 1);
    mFftOutputSize = (mPaddedLength / 2) + 1;

    // std::cout << "mPaddedLength=" << mPaddedLength // for debug
    //      << " mFftOutputSize=" << mFftOutputSize << std::endl;

    channelData.conservativeResize(channelData.rows(), mPaddedLength);
    //std::cout << "[construct] channelData.data()=" << (void*)channelData.data() << std::endl; // for debug
    std::cout << "[FreqDomainFilter ctor] resized to " << channelData.rows() // for debug
          << "x" << channelData.cols() << " mPaddedLength=" << mPaddedLength << std::endl;
    channelData.setZero();
    mSavedFFTs = Eigen::MatrixXcf::Zero(mFftOutputSize, mNumChannels);

    initializeFilterWeights(filterWeights);

    initializeFFT(channelData);
}

FrequencyDomainFIRFilter::~FrequencyDomainFIRFilter()
{
    if (mForwardFftPlan)
    {
        fftwf_destroy_plan(mForwardFftPlan);
        mForwardFftPlan = nullptr;
    }
}

void FrequencyDomainFIRFilter::apply()
{
    // std::cout << "apply addr mSavedFFTs: " << mSavedFFTs.data() << std::endl;
    std::cout << "[FreqDomainFilter] mPaddedLength=" << mPaddedLength << std::endl; // for debug

    fftwf_execute(mForwardFftPlan);
    // std::cout << "Frequency spectrum:\n" << mSavedFFTs << std::endl; // for debug
    mBeforeFilter = mSavedFFTs;
    for (int channelIndex = 0; channelIndex < mNumChannels; ++channelIndex)
    {
        mSavedFFTs.col(channelIndex) = mSavedFFTs.col(channelIndex).array() * mFilterFreq.array();
    }
}

int FrequencyDomainFIRFilter::getPaddedLength() const { return mPaddedLength; }

Eigen::MatrixXcf& FrequencyDomainFIRFilter::getFrequencyDomainData() { return mSavedFFTs; }

void FrequencyDomainFIRFilter::initializeFFT(Eigen::MatrixXf& channelData)
{
    // channelData now has the final size we need
    mForwardFftPlan = fftwf_plan_many_dft_r2c(
        1, &mPaddedLength, mNumChannels, channelData.data(), nullptr, mNumChannels, 1,
        reinterpret_cast<fftwf_complex*>(mSavedFFTs.data()), nullptr, 1, mFftOutputSize, FFTW_ESTIMATE);
}

void FrequencyDomainFIRFilter::initializeFilterWeights(const std::vector<float>& filterWeights)
{
    mFilterFreq.resize(mFftOutputSize);
    std::vector<float> paddedFilter(mPaddedLength, 0.0f);
    std::copy(filterWeights.begin(), filterWeights.end(), paddedFilter.begin());

    fftwf_plan fftFilter = fftwf_plan_dft_r2c_1d(
        mPaddedLength, paddedFilter.data(), reinterpret_cast<fftwf_complex*>(mFilterFreq.data()), FFTW_ESTIMATE);
    fftwf_execute(fftFilter);
    fftwf_destroy_plan(fftFilter);
}
