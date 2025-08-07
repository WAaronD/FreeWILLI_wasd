#include "time_domain_filters.h"

#include "filter_utility_functions.h"

FIRFilter::FIRFilter(const std::string& filterPath, Eigen::MatrixXf& channelData)
{
    mFilterWeights = readFirFilterFile(filterPath);
    // mPaddedLength = static_cast<int>(mFilterWeights.size() + channelData.cols() - 1);
    mPaddedLength = static_cast<int>(channelData.cols());
    // mFftOutputSize = (mPaddedLength / 2) + 1;

    channelData.conservativeResize(channelData.rows(), mPaddedLength);
    channelData.setZero();
    mChannelDataPtr = &channelData;
}

bool FIRFilter::apply()
{
    if (!mChannelDataPtr)
    {
        return false;
    }
    auto& data = *mChannelDataPtr;  // shape: (C, N)

    const int numChannels = data.rows();
    const int channelLength = data.cols();
    const int numTaps = int(mFilterWeights.size());  // b[0..M-1]

    //  allocate same length as input
    Eigen::MatrixXf filtered = Eigen::MatrixXf::Zero(numChannels, channelLength);

    Eigen::Map<const Eigen::VectorXf> b(mFilterWeights.data(), numTaps);

    // y[n] = sum_{k=0..M-1} b[k] * x[n-k], with x[n<0]=0
    for (int ch = 0; ch < numChannels; ++ch)
    {
        for (int n = 0; n < channelLength; ++n)
        {
            float acc = 0;
            for (int k = 0; k < numTaps; ++k)
            {
                int idx = n - k;
                if (idx < 0) break;  // zero‐pad for n<0
                acc += b[k] * data(ch, idx);
            }
            filtered(ch, n) = acc;
        }
    }

    data = std::move(filtered);
    return true;
}

// float FIRFilter::getLastDetection() const { return m_lastDetection; }