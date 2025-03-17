#pragma once
#include "../pch.h"

class IImuProcessor
{
   public:
    virtual ~IImuProcessor() {}
    virtual void processIMUData(const std::vector<uint8_t>& byteBlock) = 0;
    virtual const Eigen::Matrix3f& getRotationMatrix() = 0;
};