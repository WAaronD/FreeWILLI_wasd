#include "../../src/firmware/firmware_1240.h"

#include "gtest/gtest.h"

// Test: Insert Data Into Channel Matrix
// Test: Insert Data Into Channel Matrix
TEST(Firmware1240Test, InsertDataIntoChannelMatrix)
{
    Firmware1240 firmware;
    Eigen::MatrixXf channelMatrix(1, Firmware1240::SAMPS_PER_PACKET);  // Single packet

    // Simulated packet data: each sample is 16-bit (2 bytes)
    std::vector<std::vector<uint8_t>> dataBytes(1, std::vector<uint8_t>(Firmware1240::REQUIRED_BYTES, 0));

    // Fill in test data
    for (int i = 0; i < Firmware1240::SAMPS_PER_PACKET; ++i)
    {
        uint16_t sampleValue = static_cast<uint16_t>(i) + static_cast<uint16_t>(Firmware1240::SAMPLE_OFFSET);
        dataBytes[0][Firmware1240::HEAD_SIZE + 2 * i] = static_cast<uint8_t>(sampleValue >> 8);  // High byte
        dataBytes[0][Firmware1240::HEAD_SIZE + 2 * i + 1] = static_cast<uint8_t>(sampleValue & 0xFF);  // Low byte
    }

    firmware.insertDataIntoChannelMatrix(channelMatrix, dataBytes);

    // Check extracted values
    for (int i = 0; i < 5; ++i)  // Test the first few values
    {
        EXPECT_NEAR(channelMatrix(0, i), i, 1e-6) << "Mismatch at index " << i;
    }
}

// Test: Generate Timestamps
TEST(Firmware1240Test, GenerateTimestamp)
{
    Firmware1240 firmware;

    std::vector<std::vector<uint8_t>> dataBytes(1, std::vector<uint8_t>(Firmware1240::REQUIRED_BYTES, 0));
    dataBytes[0][0] = 24;  // Year (2024)
    dataBytes[0][1] = 3;  // Month (March)
    dataBytes[0][2] = 14;  // Day
    dataBytes[0][3] = 12;  // Hour
    dataBytes[0][4] = 30;  // Minute
    dataBytes[0][5] = 45;  // Second
    dataBytes[0][6] = 0x00;  // Microseconds (0x000F4240 = 1,000,000 us = 1 second)
    dataBytes[0][7] = 0x0F;
    dataBytes[0][8] = 0x42;
    dataBytes[0][9] = 0x40;

    auto timestamps = firmware.generateTimestamp(dataBytes, Firmware1240::NUM_CHAN);
    EXPECT_EQ(timestamps.size(), 1);

    std::tm expectedTm = {};
    expectedTm.tm_year = 2024 - 1900;
    expectedTm.tm_mon = 3 - 1;
    expectedTm.tm_mday = 14;
    expectedTm.tm_hour = 12;
    expectedTm.tm_min = 30;
    expectedTm.tm_sec = 45;

    std::time_t expected_time = std::mktime(&expectedTm);

    auto expected_timestamp =
        std::chrono::system_clock::from_time_t(expected_time) + std::chrono::microseconds(1000000);

    EXPECT_EQ(timestamps[0], expected_timestamp);
}

// Test: Throw Error for Incorrect Packet Size
TEST(Firmware1240Test, ThrowIfDataErrorsIncorrectSize)
{
    Firmware1240 firmware;

    std::vector<std::vector<uint8_t>> dataBytes(1, std::vector<uint8_t>(Firmware1240::REQUIRED_BYTES - 1, 0));
    std::vector<std::chrono::system_clock::time_point> timestamps(1, std::chrono::system_clock::now());

    bool isPreviousTimeSet = false;
    std::chrono::system_clock::time_point previousTime;

    EXPECT_THROW(
        firmware.throwIfDataErrors(
            dataBytes, Firmware1240::MICRO_INCR, isPreviousTimeSet, previousTime, timestamps,
            Firmware1240::REQUIRED_BYTES),
        std::runtime_error);
}

// Test: Throw Error for Time Increment Mismatch
TEST(Firmware1240Test, ThrowIfDataErrorsTimeIncrementMismatch)
{
    Firmware1240 firmware;

    std::vector<std::vector<uint8_t>> dataBytes(2, std::vector<uint8_t>(Firmware1240::REQUIRED_BYTES, 0));
    std::vector<std::chrono::system_clock::time_point> timestamps(2);

    timestamps[0] = std::chrono::system_clock::now();
    timestamps[1] = timestamps[0] + std::chrono::microseconds(999);  // Incorrect time increment

    bool isPreviousTimeSet = false;
    std::chrono::system_clock::time_point previousTime;

    EXPECT_THROW(
        firmware.throwIfDataErrors(
            dataBytes, Firmware1240::MICRO_INCR, isPreviousTimeSet, previousTime, timestamps,
            Firmware1240::REQUIRED_BYTES),
        std::runtime_error);
}

// Test: Correct Data Processing Without Errors
TEST(Firmware1240Test, ThrowIfDataErrorsNoErrors)
{
    Firmware1240 firmware;

    std::vector<std::vector<uint8_t>> dataBytes(2, std::vector<uint8_t>(Firmware1240::REQUIRED_BYTES, 0));
    std::vector<std::chrono::system_clock::time_point> timestamps(2);

    timestamps[0] = std::chrono::system_clock::now();
    timestamps[1] = timestamps[0] + std::chrono::microseconds(Firmware1240::MICRO_INCR);  // Correct time increment

    bool isPreviousTimeSet = false;
    std::chrono::system_clock::time_point previousTime;

    EXPECT_NO_THROW(firmware.throwIfDataErrors(
        dataBytes, Firmware1240::MICRO_INCR, isPreviousTimeSet, previousTime, timestamps,
        Firmware1240::REQUIRED_BYTES));
}
