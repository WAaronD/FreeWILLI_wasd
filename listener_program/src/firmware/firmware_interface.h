#pragma once

#include "../pch.h"
#include "imu_processor_interface.h"

/**
 * @brief interface for all firmware classes
 */
class IFirmware
{
   public:
    virtual ~IFirmware() = default;

    virtual int numChannels() const = 0;

    virtual int channelSize() const = 0;

    virtual int sampleRate() const = 0;

    virtual int numPacketsToDetect() const = 0;

    virtual int microIncre() const = 0;

    virtual int imuByteSize() const = 0;

    virtual int packetSize() const = 0;

    virtual void insertDataIntoChannelMatrix(
        Eigen::MatrixXf& channelMatrix, const std::vector<std::vector<uint8_t>>& dataBytes) const = 0;

    virtual std::vector<TimePoint> generateTimestamp(std::vector<std::vector<uint8_t>>& dataBytes) const = 0;

    virtual void throwIfDataErrors(
        const std::vector<std::vector<uint8_t>>& dataBytes, bool& isPreviousTimeSet, TimePoint& previousTime,
        const std::vector<TimePoint>& currentTime) const = 0;

    virtual IImuProcessor* getImuManager() const = 0;
};
