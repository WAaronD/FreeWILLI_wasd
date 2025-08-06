#include "time_domain_filters.h"

#include "filter_utility_functions.h"

FIRFilter::FIRFilter(const std::string& filterPath, Eigen::MatrixXf& channelData, int numChannels)
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

    const int C = data.rows();
    const int N = data.cols();
    const int M = int(mFilterWeights.size());  // b[0..M-1]

    //  allocate same length as input
    Eigen::MatrixXf filtered = Eigen::MatrixXf::Zero(C, N);

    Eigen::Map<const Eigen::VectorXf> b(mFilterWeights.data(), M);

    // y[n] = sum_{k=0..M-1} b[k] * x[n-k], with x[n<0]=0
    for (int ch = 0; ch < C; ++ch)
    {
        for (int n = 0; n < N; ++n)
        {
            float acc = 0;
            for (int k = 0; k < M; ++k)
            {
                int idx = n - k;
                if (idx < 0) break;  // zero‐pad for n<0
                acc += b[k] * data(ch, idx);
            }
            filtered(ch, n) = acc;
        }
    }

    data = std::move(filtered);
    // std::cout << data.row(0).head(5) << " … " << data.row(0).tail(5) << std::endl;
    return true;
}

// float FIRFilter::getLastDetection() const { return m_lastDetection; }