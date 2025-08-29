#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "../../src/algorithms/detectors/time_domain_detectors_factory.h"  // Adjust path accordingly
using json = nlohmann::json;
// #include <Eigen/Dense>

// Test PeakAmplitudeDetector constructor and threshold setting
TEST(PeakAmplitudeDetectorTest, ConstructorInitializesCorrectly)
{
    PeakAmplitudeDetector detector(0.5f);
    EXPECT_FLOAT_EQ(detector.getLastDetection(), 0.0f);  // Should initialize to zero
}

// Test detect method with values below and above the threshold
TEST(PeakAmplitudeDetectorTest, DetectsPeakAmplitudeCorrectly)
{
    PeakAmplitudeDetector detector(0.5f);

    Eigen::VectorXf signal(5);
    signal << 0.1f, 0.2f, 0.4f, 0.3f, 0.1f;
    EXPECT_FALSE(detector.detect(signal));  // Peak is 0.4, should be below threshold

    signal << 0.1f, 0.6f, 0.2f, 0.3f, 0.1f;
    EXPECT_TRUE(detector.detect(signal));  // Peak is 0.6, should be above threshold
}

// Test that getLastDetection() returns the correct peak amplitude
TEST(PeakAmplitudeDetectorTest, ReturnsCorrectLastDetection)
{
    PeakAmplitudeDetector detector(0.5f);

    Eigen::VectorXf signal(4);
    signal << 0.1f, 0.8f, 0.3f, 0.2f;
    detector.detect(signal);

    EXPECT_FLOAT_EQ(detector.getLastDetection(), 0.8f);
}

// Test Factory: PeakAmplitudeDetector creation
TEST(ITimeDomainDetectorFactoryTest, CreatesPeakAmplitudeDetector)
{
    json params = {{"detector", "PeakAmplitude"}, {"threshold", 0.5f}};
    auto detector = ITimeDomainDetectorFactory::create(params);
    EXPECT_NE(dynamic_cast<PeakAmplitudeDetector*>(detector.get()), nullptr);
}

// Test Factory: Throws for unknown detector type
TEST(ITimeDomainDetectorFactoryTest, ThrowsForUnknownDetectorType)
{
    json params = {{"detector", "InvalidDetector"}, {"threshold", 0.5f}};
    EXPECT_THROW(ITimeDomainDetectorFactory::create(params), std::invalid_argument);
}