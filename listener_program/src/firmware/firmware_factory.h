#pragma once

#include "firmware_1240.h"
#include "firmware_1240_imu.h"

class FirmwareFactory
{
   public:
    static std::unique_ptr<const IFirmware> create(const std::string& firmwareToUse)
    {
        if (firmwareToUse == "1240_imu")
        {
            return std::make_unique<const Firmware1240IMU>();
        }
        else if (firmwareToUse == "1240")
        {
            return std::make_unique<const Firmware1240>();
        }
        else
        {
            std::cerr << "Specified firmware version not recognized \n";
            std::exit(1);
        }
    }
};