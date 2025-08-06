#include "ecompass.h"

/**
 * @brief Calculates the rotation matrix from accelerometer and magnetometer readings.
 *
 * This function computes a rotation matrix using the normalized accelerometer and magnetometer
 * vectors. The accelerometer provides the gravity direction, and the magnetometer provides the
 * magnetic field direction. The resulting rotation matrix describes the orientation of the device.
 *
 * @param accelerometerData A 3D vector representing normalized accelerometer readings (gravity direction).
 * @param magnetometerData A 3D vector representing normalized magnetometer readings (magnetic field direction).
 * @return Eigen::Matrix3f The calculated rotation matrix, where:
 *         - Column 0 is the North vector (Y-axis),
 *         - Column 1 is the East vector (X-axis),
 *         - Column 2 is the Down vector (Z-axis).
 */
Eigen::Matrix3f ECompass::process(const Eigen::Vector3f& accelerometerData, const Eigen::Vector3f& magnetometerData)
{
    Eigen::Vector3f gravityDirection = accelerometerData.normalized();

    Eigen::Vector3f magneticFieldDirection = magnetometerData.normalized();

    Eigen::Vector3f eastVector = (gravityDirection.cross(magneticFieldDirection)).normalized();

    Eigen::Vector3f northVector = eastVector.cross(gravityDirection);

    // Construct the rotation matrix
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix.col(0) = northVector;  // Y-axis (North)
    rotationMatrix.col(1) = eastVector;  // X-axis (East)
    rotationMatrix.col(2) = gravityDirection;  // Z-axis (Down)

    return rotationMatrix;
}