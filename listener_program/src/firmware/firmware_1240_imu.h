#pragma once

#include "firmware_1240.h"
#include "imu_processor_1240.h"

/**
 * @brief  Class for firmware 1240 configuration with IMU data, providing constants and utility methods.
 */
class Firmware1240IMU : public Firmware1240
{
   public:
    Firmware1240IMU() : mImuByteSize(32) { imuManager = std::make_unique<ImuProcessor1240>(mImuByteSize); }

    constexpr int imuByteSize() const override { return mImuByteSize; }

   private:
    int mImuByteSize;
};
