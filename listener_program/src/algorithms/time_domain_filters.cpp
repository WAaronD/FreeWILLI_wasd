#include "time_domain_filters.h"

#include "filter_utility_functions.h"

FIRFilter::FIRFilter(const std::string& filterPath, Eigen::MatrixXf& channelData, bool Ch1Only)
{
    mFilterWeights = readFirFilterFile(filterPath);
    // mPaddedLength = static_cast<int>(mFilterWeights.size() + channelData.cols() - 1);
    mPaddedLength = static_cast<int>(channelData.cols());
    // mFftOutputSize = (mPaddedLength / 2) + 1;

    channelData.conservativeResize(channelData.rows(), mPaddedLength);
    channelData.setZero();
    mChannelDataPtr = &channelData;

    mCh1Only = Ch1Only;
}

bool FIRFilter::apply()
{
    if (!mChannelDataPtr)
    {
        return false;
    }
    auto& data = *mChannelDataPtr;  // shape: (C, N)
    
    const int numChannels = mCh1Only ? 1 : data.rows(); // Set num channels to 1 Ch1Only specified, otherwise use actual # of channels
    const int channelLength = data.cols();
    const int numTaps = int(mFilterWeights.size());  // b[0..M-1]

    //  allocate same length as input
    Eigen::MatrixXf filtered = Eigen::MatrixXf::Zero(numChannels, channelLength);

    Eigen::Map<const Eigen::VectorXf> b(mFilterWeights.data(), numTaps);

    // y[n] = sum_{k=0..M-1} b[k] * x[n-k], with x[n<0]=0
    for (int ch = 0; ch < numChannels; ++ch) // Loop through channels
    {
        for (int n = 0; n < channelLength; ++n) // Loop through samples
        {
            float acc = 0;
            for (int k = 0; k < numTaps; ++k) // Loop through filter taps
            {
                int idx = n - k;
                if (idx < 0) break;  // zero‐pad for n<0
                acc += b[k] * data(ch, idx);
            }
            filtered(ch, n) = acc;
        }
    }

    // data = std::move(filtered); // Wrong! Do NOT use std::move
    // data = filtered; // Fixed (Reason: copying keeps the buffer address stable for the FFTW plan; move doesn't guarantee this)
    data.topRows(numChannels) = filtered; // Copy filtered data back to original matrix, only for channels that were processed
    // std::cout << data.leftCols(100) << std::endl; // debug
    return true;
}

// float FIRFilter::getLastDetection() const { return m_lastDetection; }