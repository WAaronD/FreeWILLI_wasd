#pragma once

#include "firmware_1240.h"
#include "firmware_1240_imu.h"

class FirmwareFactory
{
   public:
    static std::unique_ptr<const Firmware1240> create(bool useImu)
    {
        if (useImu)
        {
            return std::make_unique<const Firmware1240IMU>();
        }
        else
        {
            return std::make_unique<const Firmware1240>();
        }
    }
};