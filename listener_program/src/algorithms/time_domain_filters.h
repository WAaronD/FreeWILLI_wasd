#pragma once
#include "../pch.h"

class ITimeDomainFilter
{
   public:
    virtual ~ITimeDomainFilter() = default;
    virtual bool apply() = 0;
};

class FIRFilter : public ITimeDomainFilter
{
   public:
    explicit FIRFilter(const std::string& filterPath, Eigen::MatrixXf& channelData, bool Ch1Only = false);

    bool apply() override;

   private:
    int mPaddedLength;
    std::vector<float> mFilterWeights;
    Eigen::MatrixXf* mChannelDataPtr = nullptr;  // pointer to the channel data
    bool mCh1Only = false;  // if true, only apply filter to channel 1
};