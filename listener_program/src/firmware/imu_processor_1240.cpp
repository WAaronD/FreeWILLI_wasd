#include "imu_processor_1240.h"
ImuProcessor1240::ImuProcessor1240(int IMU_BYTE_SIZE)
    : mImuByteSize(IMU_BYTE_SIZE), mRotationMatrix(Eigen::Matrix3f::Identity())
{
}

const Eigen::Matrix3f& ImuProcessor1240::getRotationMatrix() { return mRotationMatrix; }

/**
 * @brief Updates the rotation matrix based on the latest IMU data bytes.
 *
 * This function reads raw IMU data from a vector of bytes, constructs a data vector,
 * calibrates the IMU sensor readings, and computes the rotation matrix using
 * accelerometer and magnetometer data.
 *
 * @param dataBytes     A vector of bytes containing the raw IMU data.
 * @param imuByteSize   The size in bytes of each IMU data packet.
 */
void ImuProcessor1240::setRotationMatrix(Eigen::VectorXf& imuData)
{
    calibrateImuData(imuData);

    Eigen::Vector3f magnetometerData = imuData.segment<mDataWidth>(mMagnetometerDataIndex);

    Eigen::Vector3f gyroscopeData = imuData.segment<mDataWidth>(mGyroscopeDataIndex);

    Eigen::Vector3f accelerometerData = imuData.segment<mDataWidth>(mAccelerometerDataIndex);

    mRotationMatrix = calculateRotationMatrix.process(accelerometerData, magnetometerData);
}

/**
 * @brief Calibrates IMU data using predefined calibration constants.
 *
 * @param imuData A vector containing raw IMU data.
 */
void ImuProcessor1240::calibrateImuData(Eigen::VectorXf& imuData)
{
    // Apply element-wise calibration to each sensors data
    imuData.segment<mDataWidth>(mMagnetometerDataIndex) =
        imuData.segment<mDataWidth>(mMagnetometerDataIndex).cwiseProduct(mMagnetometerCalibration);

    imuData.segment<mDataWidth>(mGyroscopeDataIndex) *= mGyroscopeCalibration;

    imuData.segment<mDataWidth>(mAccelerometerDataIndex) *= mAccelerometerCalibration;
}
/**
 * @brief Extracts and parses IMU data from a byte block.
 *
 * This function reads a block of bytes, validates the header, and extracts IMU data including
 * frame synchronization, timestamp, magnetometer, gyroscope, and accelerometer readings.
 * If the header is invalid, it returns an empty optional.
 *
 * @param byteBlock A vector of bytes containing IMU data.
 * @param imuByteSize The expected size of the IMU data block in bytes.
 * @return std::optional<Eigen::VectorXf> Parsed IMU data in a 20-element vector, or std::nullopt if the header is
 * invalid.
 */

void ImuProcessor1240::processIMUData(const std::vector<uint8_t>& byteBlock)
{
    Eigen::VectorXf imuData(20);  // Reserve space for 20 elements
    std::vector<uint8_t> block(byteBlock.end() - mImuByteSize, byteBlock.end());

    // Check for valid header ('I' and 'M')
    if (block[0] == 'I' && block[1] == 'M')
    {
        uint16_t frameSync = *reinterpret_cast<const uint16_t*>(&block[2]);
        const uint8_t* timestamp = &block[4];
        uint16_t milliseconds = *reinterpret_cast<const uint16_t*>(&block[10]);
        uint16_t counter = *reinterpret_cast<const uint16_t*>(&block[12]);
        const int16_t* magnetometerData = reinterpret_cast<const int16_t*>(&block[14]);
        const int16_t* gyroscopeData = reinterpret_cast<const int16_t*>(&block[20]);
        const int16_t* accelerometerData = reinterpret_cast<const int16_t*>(&block[26]);

        imuData(0) = 'I';
        imuData(1) = 'M';
        imuData(2) = frameSync;

        for (int i = 0; i < 6; ++i) imuData(3 + i) = timestamp[i];

        imuData(9) = milliseconds;
        imuData(10) = counter;

        for (int i = 0; i < mDataWidth; ++i)
        {
            imuData(mMagnetometerDataIndex + i) = magnetometerData[i];
            imuData(mGyroscopeDataIndex + i) = gyroscopeData[i];
            imuData(mAccelerometerDataIndex + i) = accelerometerData[i];
        }

        setRotationMatrix(imuData);
    }
}
