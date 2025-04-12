#pragma once

#include "firmware_1240.h"
#include "imu_processor_1240.h"

/**
 * @brief  Class for firmware 1240 configuration with IMU data, providing constants and utility methods.
 */
class Firmware1240IMU : public Firmware1240
{
   public:
    int imuByteSize() const override { return mImuByteSize; }

    IImuProcessor* getImuManager() const override { return imuManager.get(); }

   private:
    static constexpr int mImuByteSize = 32;
    std::unique_ptr<IImuProcessor> imuManager = std::make_unique<ImuProcessor1240>(mImuByteSize);
};
