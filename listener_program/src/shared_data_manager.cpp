#include "shared_data_manager.h"

using namespace std::chrono_literals;
/**
 * @brief Push data into the shared buffer in a thread-safe manner.
 * @param data Reference to a vector of bytes representing the incoming data.
 * @return The size of the data buffer after pushing.
 */
int SharedDataManager::pushDataToBuffer(const std::vector<uint8_t>& data)
{
    std::lock_guard<std::mutex> lock(dataBufferLock);
    dataBuffer.push(data);
    return static_cast<int>(dataBuffer.size());
}

/**
 * @brief Waits until the required number of packets are available and retrieves them.
 * @param dataBytes Destination vector for the retrieved packets.
 * @param numPacksToGet Number of packets to fetch from the buffer.
 * @return None (blocks until data is available).
 */
void SharedDataManager::waitForData(std::vector<std::vector<uint8_t>>& dataBytes, int numPacksToGet)
{
    while (true)
    {
        if (popDataFromBuffer(dataBytes, numPacksToGet))
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
}

/**
 * @brief Pops up to numPacksToGet packets from the shared buffer in a thread-safe manner.
 * @param data Destination vector for the popped packets.
 * @param numPacksToGet Number of packets to pop.
 * @return True if enough packets were popped, otherwise false.
 */
bool SharedDataManager::popDataFromBuffer(std::vector<std::vector<uint8_t>>& data, int numPacksToGet)
{
    std::lock_guard<std::mutex> lock(dataBufferLock);
    if (dataBuffer.size() >= numPacksToGet)
    {
        for (int i = 0; i < numPacksToGet; i++)
        {
            data[i] = dataBuffer.front();
            dataBuffer.pop();
        }
        return true;
    }
    return false;
}
