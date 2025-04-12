#pragma once

#include "../pch.h"
#include "firmware_interface.h"

/**
 * @brief Base class for firmware 1240 configuration, providing constants and utility methods.
 */
class Firmware1240 : public IFirmware
{
   protected:
    const int NUM_CHAN = 4;
    const int SAMPLE_RATE = 1e5;
    const int MICRO_INCR = 1240;  // microseconds between data packets
    const int SAMPS_PER_CHANNEL = 124;  // Samples per packet per channel
    const float TIME_WINDOW = 0.01;  // Fraction of a second for cross-correlation
    const int HEAD_SIZE = 12;  // Packet head size (bytes)
    const int BYTES_PER_SAMP = 2;  // Bytes per sample
    const float SAMPLE_OFFSET = 32768.0f;  // value used to convert unsigned 16 bit to signed

    const int SAMPS_PER_PACKET = SAMPS_PER_CHANNEL * NUM_CHAN;  // Samples per packet
    const int DATA_SIZE = SAMPS_PER_PACKET * BYTES_PER_SAMP;  // Packet data size (bytes)
    const int REQUIRED_BYTES = DATA_SIZE + HEAD_SIZE;
    const int DATA_BYTES_PER_CHANNEL = SAMPS_PER_CHANNEL * BYTES_PER_SAMP;  // Data bytes per channel

   public:
    int numChannels() const override { return NUM_CHAN; }

    int sampleRate() const override { return SAMPLE_RATE; }

    int microIncre() const override { return MICRO_INCR; }

    int numPacketsToDetect() const override { return static_cast<int>(TIME_WINDOW * SAMPLE_RATE / SAMPS_PER_CHANNEL); }

    int channelSize() const override { return numPacketsToDetect() * SAMPS_PER_CHANNEL; }

    int imuByteSize() const override { return 0; }

    int packetSize() const override { return DATA_SIZE + HEAD_SIZE + imuByteSize(); }

    void insertDataIntoChannelMatrix(
        Eigen::MatrixXf& channelMatrix, const std::vector<std::vector<uint8_t>>& dataBytes) const override;

    std::vector<TimePoint> generateTimestamp(std::vector<std::vector<uint8_t>>& dataBytes) const override;

    void throwIfDataErrors(
        const std::vector<std::vector<uint8_t>>& dataBytes, bool& isPreviousTimeSet, TimePoint& previousTime,
        const std::vector<TimePoint>& currentTime) const override;

    IImuProcessor* getImuManager() const override { return nullptr; }
};
