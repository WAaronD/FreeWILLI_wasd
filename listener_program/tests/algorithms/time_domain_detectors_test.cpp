#include "../../src/algorithms/time_domain_detectors.h"  // Adjust path accordingly

#include <gtest/gtest.h>

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

// Test NoTimeDomainDetector behavior
TEST(NoTimeDomainDetectorTest, AlwaysReturnsTrue)
{
    NoTimeDomainDetector detector;

    Eigen::VectorXf signal(4);
    signal << 0.1f, 0.5f, 0.3f, 0.2f;

    EXPECT_TRUE(detector.detect(signal));  // Should always return true
}

// Test NoTimeDomainDetector's last detection value
TEST(NoTimeDomainDetectorTest, ReturnsPeakAmplitude)
{
    NoTimeDomainDetector detector;

    Eigen::VectorXf signal(5);
    signal << 0.2f, 0.9f, 0.3f, 0.4f, 0.1f;
    detector.detect(signal);

    EXPECT_FLOAT_EQ(detector.getLastDetection(), 0.9f);
}

// Test Factory: PeakAmplitudeDetector creation
TEST(ITimeDomainDetectorFactoryTest, CreatesPeakAmplitudeDetector)
{
    auto detector = ITimeDomainDetectorFactory::create("PeakAmplitude", 0.5f);
    EXPECT_NE(dynamic_cast<PeakAmplitudeDetector*>(detector.get()), nullptr);
}

// Test Factory: NoTimeDomainDetector creation
TEST(ITimeDomainDetectorFactoryTest, CreatesNoTimeDomainDetector)
{
    auto detector = ITimeDomainDetectorFactory::create("None", 0.0f);
    EXPECT_NE(dynamic_cast<NoTimeDomainDetector*>(detector.get()), nullptr);
}

// Test Factory: Throws for unknown detector type
TEST(ITimeDomainDetectorFactoryTest, ThrowsForUnknownDetectorType)
{
    EXPECT_THROW(ITimeDomainDetectorFactory::create("InvalidDetector", 0.5f), std::invalid_argument);
}
