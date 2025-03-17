#include "../../src/io/output_manager.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(OutputManagerTest, AppendToBufferAndFlush)
{
    OutputManager outputManager(std::chrono::seconds(10), false, "logs/");

    Eigen::VectorXf tdoaVector(3);
    tdoaVector << 1.0, 2.0, 3.0;

    Eigen::VectorXf xCorrAmps(3);
    xCorrAmps << 0.5, 0.6, 0.7;

    TimePoint peakTime = std::chrono::system_clock::now();

    // Append data to buffer
    outputManager.appendToBuffer(10.0, 0.1, 0.2, 0.3, tdoaVector, xCorrAmps, peakTime);

    // Call flush (this should trigger writing)
    EXPECT_NO_THROW(outputManager.flushBufferIfNecessary());

    // The test succeeds if no exceptions are thrown
}
TEST(OutputManagerTest, ProgramTerminatesWhenTimeExceeded)
{
    std::cout << "Current Working Directory: " << std::filesystem::current_path() << std::endl;

    std::string outputDir = "../deployment_files/";
    OutputManager outputManager(std::chrono::seconds(1), false, outputDir);

    const int numChannels = 4;
    const TimePoint timestamp = std::chrono::system_clock::now();
    std::string filePath = outputDir + convertTimePointToString(timestamp);

    // 🔹 Ensure the file is deleted after the test
    auto cleanup = [&]()
    {
        if (std::filesystem::exists(filePath))
        {
            std::filesystem::remove(filePath);
            std::cout << "Deleted test output file: " << filePath << std::endl;
        }
    };
    std::shared_ptr<void> cleanupGuard(nullptr, [&](void*) { cleanup(); });

    outputManager.initializeOutputFile(timestamp, numChannels);

    std::this_thread::sleep_for(std::chrono::seconds(2));  // Simulate elapsed time

    // 🔹 Capture std::cout output before the process exits
    testing::internal::CaptureStdout();
    EXPECT_EXIT(outputManager.terminateProgramIfNecessary(), ::testing::ExitedWithCode(0), ".*");
    std::string output = testing::internal::GetCapturedStdout();

    EXPECT_NE(output.find("Terminating program"), std::string::npos);
}

TEST(OutputManagerTest, WriteErrorDataToCerr)
{
    OutputManager outputManager(std::chrono::seconds(10), false, "logs/");

    std::vector<TimePoint> errorTimestamps = {std::chrono::system_clock::now()};
    std::vector<std::vector<uint8_t>> erroredDataBytes = {{0xAA, 0xBB, 0xCC}};

    testing::internal::CaptureStderr();
    outputManager.writeDataToCerr(errorTimestamps, erroredDataBytes);
    std::string output = testing::internal::GetCapturedStderr();

    EXPECT_NE(output.find("Timestamps of data causing error"), std::string::npos);
    EXPECT_NE(output.find("Errored bytes of last packets"), std::string::npos);
}