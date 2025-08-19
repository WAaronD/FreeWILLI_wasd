#include "../src/utils.h"

#include <fstream>

#include "gtest/gtest.h"

// Test: Convert TimePoint to String
TEST(UtilsTest, ConvertTimePointToString)
{
    // Define a specific time point (e.g., March 14, 2025, 15:09:26.123456 UTC)
    std::tm timeStruct = {};
    timeStruct.tm_year = 2025 - 1900;  // Year since 1900
    timeStruct.tm_mon = 2;  // March (0-based)
    timeStruct.tm_mday = 14;
    timeStruct.tm_hour = 15;  // UTC hour
    timeStruct.tm_min = 9;
    timeStruct.tm_sec = 26;

    // Use gmtime to avoid timezone issues
    std::time_t timeVal = timegm(&timeStruct);
    TimePoint timePoint = std::chrono::system_clock::from_time_t(timeVal) + std::chrono::microseconds(123456);

    std::string formattedTime = convertTimePointToString(timePoint);

    // Expected format: YYMMDD_HHMMSS_microseconds (always in UTC)
    EXPECT_EQ(formattedTime, "250314_150926_123456");
}
