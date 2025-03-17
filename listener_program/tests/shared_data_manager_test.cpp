
#include "../src/shared_data_manager.h"  // Adjust the include path

#include <gtest/gtest.h>

// Test pushing data into the buffer
TEST(SharedDataManagerTest, PushDataIncreasesBufferSize)
{
    SharedDataManager manager;
    std::vector<uint8_t> testData = {1, 2, 3, 4};

    int bufferSize = manager.pushDataToBuffer(testData);
    EXPECT_EQ(bufferSize, 1);
}

// Test waiting for data in a separate thread
TEST(SharedDataManagerTest, WaitForDataRetrievesPackets)
{
    SharedDataManager manager;

    int sizeInfo = manager.pushDataToBuffer({100, 101, 102});

    // Try retrieving data
    std::vector<std::vector<uint8_t>> retrievedData;
    retrievedData.resize(1);

    manager.waitForData(retrievedData, 1);  // Blocks until at least 1 packet is available

    // Ensure the retrieved data is correct
    EXPECT_EQ(retrievedData.size(), 1);
    EXPECT_EQ(retrievedData[0], (std::vector<uint8_t>{100, 101, 102}));
}

// Test that `waitForData` blocks until data is available
TEST(SharedDataManagerTest, WaitForDataBlocksAndRetrievesData)
{
    SharedDataManager manager;
    std::vector<std::vector<uint8_t>> retrievedData;
    retrievedData.resize(1);

    // Start a thread that pushes data after a delay
    std::thread producer(
        [&manager]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            manager.pushDataToBuffer({200, 201, 202});
        });

    std::thread consumer([&manager, &retrievedData]() { manager.waitForData(retrievedData, 1); });

    producer.join();
    consumer.join();

    ASSERT_EQ(retrievedData.size(), 1);
    EXPECT_EQ(retrievedData[0], (std::vector<uint8_t>{200, 201, 202}));
}

/*
// Test that an empty buffer correctly fails to pop data
TEST(SharedDataManagerTest, PopDataFailsIfBufferIsEmpty)
{
    SharedDataManager manager;
    std::vector<std::vector<uint8_t>> retrievedData;
    EXPECT_FALSE(manager.popDataFromBuffer(retrievedData, 1));
}
*/

// Test pushing multiple items and retrieving them in the correct order
/*
TEST(SharedDataManagerTest, PushAndPopMaintainsFIFOOrder)
{
    SharedDataManager manager;
    manager.pushDataToBuffer({1, 2, 3});
    manager.pushDataToBuffer({4, 5, 6});
    manager.pushDataToBuffer({7, 8, 9});

    std::vector<std::vector<uint8_t>> retrievedData;
    manager.popDataFromBuffer(retrievedData, 3);

    ASSERT_EQ(retrievedData.size(), 3);
    EXPECT_EQ(retrievedData[0], (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(retrievedData[1], (std::vector<uint8_t>{4, 5, 6}));
    EXPECT_EQ(retrievedData[2], (std::vector<uint8_t>{7, 8, 9}));
}
*/

// Test atomic flag `errorOccurred`
TEST(SharedDataManagerTest, ErrorOccurredFlagWorks)
{
    SharedDataManager manager;
    EXPECT_FALSE(manager.errorOccurred);

    manager.errorOccurred = true;
    EXPECT_TRUE(manager.errorOccurred);
}

// Test atomic counter `detectionCounter`
TEST(SharedDataManagerTest, DetectionCounterIncrements)
{
    SharedDataManager manager;
    EXPECT_EQ(manager.detectionCounter, 0);

    manager.detectionCounter++;
    EXPECT_EQ(manager.detectionCounter, 1);
}
