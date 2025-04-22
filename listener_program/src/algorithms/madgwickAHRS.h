#pragma once

#include "../pch.h"

class MadgwickAHRS
{
   public:
    MadgwickAHRS(float sampleFrequency, float betaGain);

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;

    void printQuaternion() const;

   private:
    float beta;
    float sampleFreq;
    float q0, q1, q2, q3;

    static float invSqrt(float x);
};