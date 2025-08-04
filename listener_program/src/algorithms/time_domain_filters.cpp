#include "time_domain_filters.h"

#include "filter_utility_functions.h"

FIRFilter::FIRFilter(const std::string& filterPath, Eigen::MatrixXf& channelData, int numChannels)
{
    auto mFilterWeights = readFirFilterFile(filterPath);
    mPaddedLength = static_cast<int>(mFilterWeights.size() + channelData.cols() - 1);
    // mFftOutputSize = (mPaddedLength / 2) + 1;

    channelData.conservativeResize(channelData.rows(), mPaddedLength);
    channelData.setZero();
    mChannelDataPtr = &channelData;
}

bool FIRFilter::apply()
{
    if (!mChannelDataPtr) return false;
    auto& data = *mChannelDataPtr;

    int numChannels = data.rows();
    int dataLength = data.cols();
    int filterLength = static_cast<int>(mFilterWeights.size());

    Eigen::MatrixXf filtered = Eigen::MatrixXf::Zero(numChannels, dataLength - filterLength + 1);

    Eigen::Map<const Eigen::VectorXf> kernel(mFilterWeights.data(), filterLength);

    // Apply FIR filter to each channel (row) using convolution
    for (int ch = 0; ch < numChannels; ++ch)
    {
        for (int i = 0; i <= dataLength - filterLength; ++i)
        {
            filtered(ch, i) = data.row(ch).segment(i, filterLength).dot(kernel);
        }
    }

    // Resize data to match filtered output size and store it
    data = std::move(filtered);

    // Optionally, update detection metric
    // mLastDetection = data.cwiseAbs().maxCoeff();  // just an example metric

    return true;
}

// float FIRFilter::getLastDetection() const { return m_lastDetection; }