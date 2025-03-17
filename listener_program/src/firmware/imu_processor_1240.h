#pragma once
#include "../algorithms/ecompass.h"
#include "../pch.h"
#include "imu_processor_interface.h"

/**
 * @class ImuProcessor
 * @brief A class for processing IMU data.
 *
 * This class provides methods to extract, calibrate, and process IMU data from raw byte streams,
 * and compute the rotation matrix based on accelerometer and magnetometer readings.
 */
class ImuProcessor1240 : public IImuProcessor
{
   public:
    explicit ImuProcessor1240(int IMU_BYTE_SIZE);

    const Eigen::Matrix3f& getRotationMatrix() override;

    void processIMUData(const std::vector<uint8_t>& byteBlock) override;

   private:
    // Calibration constants
    int mImuByteSize;

    Eigen::Matrix3f mRotationMatrix;

    static constexpr int mDataWidth = 3;
    static constexpr int mMagnetometerDataIndex = 11;
    static constexpr int mGyroscopeDataIndex = 14;
    static constexpr int mAccelerometerDataIndex = 17;

    const float mAccelerometerCalibration = 2.0f / 32768.0f;
    const float mGyroscopeCalibration = 2000.0f / 32768.0f;
    const Eigen::Vector3f mMagnetometerCalibration{1150.0f / 32768.0f, 1150.0f / 32768.0f, 2250.0f / 32768.0f};

    void calibrateImuData(Eigen::VectorXf& imuData);
    void setRotationMatrix(Eigen::VectorXf& imuData);

    ECompass calculateRotationMatrix;
};
