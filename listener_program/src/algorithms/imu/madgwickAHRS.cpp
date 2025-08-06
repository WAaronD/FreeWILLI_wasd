#include "madgwickAHRS.h"

MadgwickAHRS::MadgwickAHRS(float sampleFrequency, float betaGain)
    : beta(betaGain), sampleFreq(sampleFrequency), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f)
{
}

void MadgwickAHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    // Use IMU algorithm if magnetometer data invalid
    if (mx == 0.0f && my == 0.0f && mz == 0.0f)
    {
        updateImu(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change from gyroscope
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalize accelerometer
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalize magnetometer
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Precompute quaternion products
        float twoQ0 = 2.0f * q0;
        float twoQ1 = 2.0f * q1;
        float twoQ2 = 2.0f * q2;
        float twoQ3 = 2.0f * q3;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q3 = q2 * q3;

        // Reference field
        float twoQ0Mx = twoQ0 * mx;
        float twoQ0My = twoQ0 * my;
        float twoQ0Mz = twoQ0 * mz;
        float twoQ1Mx = twoQ1 * mx;

        float hx = mx * q0q0 - twoQ0My * q3 + twoQ0Mz * q2 + mx * q1q1 + twoQ1 * my * q2 + twoQ1 * mz * q3 - mx * q2q2 -
                   mx * q3q3;
        float hy = twoQ0Mx * q3 + my * q0q0 - twoQ0Mz * q1 + twoQ1Mx * q2 - my * q1q1 + my * q2q2 + twoQ2 * mz * q3 -
                   my * q3q3;
        float twoBx = std::sqrt(hx * hx + hy * hy);
        float twoBz = -twoQ0Mx * q2 + twoQ0My * q1 + mz * q0q0 + twoQ1Mx * q3 - mz * q1q1 + twoQ2 * my * q3 -
                      mz * q2q2 + mz * q3q3;
        float fourBx = 2.0f * twoBx;
        float fourBz = 2.0f * twoBz;

        // Gradient descent corrective step
        float s0 = -twoQ2 * (2.0f * q1q3 - twoQ0 * q2 - ax) + twoQ1 * (2.0f * q0q1 + twoQ2 * q3 - ay) -
                   twoBz * q2 * (fourBx * (0.5f - q2q2 - q3q3) + fourBz * (q1q3 - q0q2) - mx) +
                   (-fourBx * q3 + fourBz * q1) * (fourBx * (q1q2 - q0q3) + fourBz * (q0q1 + q2q3) - my) +
                   fourBx * q2 * (fourBx * (q0q2 + q1q3) + fourBz * (0.5f - q1q1 - q2q2) - mz);

        float s1 = twoQ3 * (2.0f * q1q3 - twoQ0 * q2 - ax) + twoQ0 * (2.0f * q0q1 + twoQ2 * q3 - ay) -
                   4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                   twoBz * q3 * (fourBx * (0.5f - q2q2 - q3q3) + fourBz * (q1q3 - q0q2) - mx) +
                   (twoBx * q2 + twoBz * q0) * (fourBx * (q1q2 - q0q3) + fourBz * (q0q1 + q2q3) - my) +
                   (twoBx * q3 - fourBz * q1) * (fourBx * (q0q2 + q1q3) + fourBz * (0.5f - q1q1 - q2q2) - mz);

        float s2 = -twoQ0 * (2.0f * q1q3 - twoQ0 * q2 - ax) + twoQ3 * (2.0f * q0q1 + twoQ2 * q3 - ay) -
                   4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                   (-2.0f * fourBx * q2 - twoBz * q0) * (fourBx * (0.5f - q2q2 - q3q3) + fourBz * (q1q3 - q0q2) - mx) +
                   (twoBx * q1 + twoBz * q3) * (fourBx * (q1q2 - q0q3) + fourBz * (q0q1 + q2q3) - my) +
                   (twoBx * q0 - fourBz * q2) * (fourBx * (q0q2 + q1q3) + fourBz * (0.5f - q1q1 - q2q2) - mz);

        float s3 = twoQ1 * (2.0f * q1q3 - twoQ0 * q2 - ax) + twoQ2 * (2.0f * q0q1 + twoQ2 * q3 - ay) +
                   (-twoBx * q0 + twoBz * q2) * (fourBx * (q1q2 - q0q3) + fourBz * (q0q1 + q2q3) - my) +
                   (-twoBx * q0 + twoBz * q2) * (fourBx * (q0q2 + q1q3) + fourBz * (0.5f - q1q1 - q2q2) - mz) +
                   (-fourBx * q3 + twoBz * q1) * (fourBx * (0.5f - q2q2 - q3q3) + fourBz * (q1q3 - q0q2) - mx);

        // Normalize correction
        float recipStepNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipStepNorm;
        s1 *= recipStepNorm;
        s2 *= recipStepNorm;
        s3 *= recipStepNorm;

        // Apply feedback
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    performQuaternionIntegration(qDot0, qDot1, qDot2, qDot3);
}

void MadgwickAHRS::updateImu(float gx, float gy, float gz, float ax, float ay, float az)
{
    // Gyro rate
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float twoQ0 = 2.0f * q0;
        float twoQ1 = 2.0f * q1;
        float twoQ2 = 2.0f * q2;
        float twoQ3 = 2.0f * q3;
        float fourQ0 = 4.0f * q0;
        float fourQ1 = 4.0f * q1;
        float fourQ2 = 4.0f * q2;
        float eightQ1 = 8.0f * q1;
        float eightQ2 = 8.0f * q2;

        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // Gradient descent step
        float s0 = fourQ0 * q2q2 + twoQ2 * ax + fourQ0 * q1q1 - twoQ1 * ay;
        float s1 = fourQ1 * q3q3 - twoQ3 * ax + 4.0f * q0q0 * q1 - twoQ0 * ay - fourQ1 + eightQ1 * q1q1 +
                   eightQ1 * q2q2 + fourQ1 * az;
        float s2 = 4.0f * q0q0 * q2 + twoQ0 * ax + fourQ2 * q3q3 - twoQ3 * ay - fourQ2 + eightQ2 * q1q1 +
                   eightQ2 * q2q2 + fourQ2 * az;
        float s3 = 4.0f * q1q1 * q3 - twoQ1 * ax + 4.0f * q2q2 * q3 - twoQ2 * ay;

        float recipStepNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipStepNorm;
        s1 *= recipStepNorm;
        s2 *= recipStepNorm;
        s3 *= recipStepNorm;

        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    performQuaternionIntegration(qDot0, qDot1, qDot2, qDot3);
}

void MadgwickAHRS::performQuaternionIntegration(float qDot0, float qDot1, float qDot2, float qDot3)
{
    float invSampleFreq = 1.0f / sampleFreq;
    q0 += qDot0 * invSampleFreq;
    q1 += qDot1 * invSampleFreq;
    q2 += qDot2 * invSampleFreq;
    q3 += qDot3 * invSampleFreq;

    // Normalize quaternion
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float MadgwickAHRS::invSqrt(float x)
{
    float halfX = 0.5f * x;
    float y = x;
    auto i = *reinterpret_cast<std::int32_t*>(&y);
    i = 0x5f3759df - (i >> 1);
    y = *reinterpret_cast<float*>(&i);
    return y * (1.5f - (halfX * y * y));
}

void MadgwickAHRS::getQuaternion(float& q0Out, float& q1Out, float& q2Out, float& q3Out) const
{
    q0Out = q0;
    q1Out = q1;
    q2Out = q2;
    q3Out = q3;
}

void MadgwickAHRS::printQuaternion() const
{
    std::cout << "Quaternion(w=" << q0 << ", x=" << q1 << ", y=" << q2 << ", z=" << q3 << ")\n";
}