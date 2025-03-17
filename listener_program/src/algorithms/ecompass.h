#pragma once
#include "../pch.h"

class ECompass
{
   public:
    Eigen::Matrix3f process(const Eigen::Vector3f& accelerometerData, const Eigen::Vector3f& magnetometerData);
};