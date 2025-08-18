#include "../src/utils.h"

#include <fstream>

#include "gtest/gtest.h"

// Test: JSON Parsing with Missing Fields
TEST(UtilsTest, ParseJsonConfigMissingField)
{
    std::string tempJsonFile = "temp_config_missing.json";
    std::ofstream file(tempJsonFile);
    file << R"({
        "IPAddress": "192.168.1.1",
        "Port": 5000
    })";
    file.close();

    EXPECT_THROW(parseJsonConfig(tempJsonFile), std::exception);

    std::remove(tempJsonFile.c_str());
}

// Test: JSON Parsing with Invalid File
TEST(UtilsTest, ParseJsonConfigInvalidFile) { EXPECT_THROW(parseJsonConfig("non_existent.json"), std::runtime_error); }

// Test: Print Mode Output
TEST(UtilsTest, PrintModeTest)
{
    testing::internal::CaptureStdout();
    printMode();
    std::string output = testing::internal::GetCapturedStdout();

#ifdef DEBUG
    EXPECT_EQ(output, "Running Debug Mode\n");
#else
    EXPECT_EQ(output, "Running Release Mode\n");
#endif
}

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
