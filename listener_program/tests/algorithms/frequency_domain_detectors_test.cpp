#include "../../src/algorithms/frequency_domain_detectors_factory.h"
#include "../../src/pch.h"
#include "gtest/gtest.h"

/*
    the following are tests for normalizeDoa function
*/

// Test: Detection when the average magnitude is above the threshold
TEST(AverageMagnitudeDetectorTest, DetectsWhenAboveThreshold)
{
    float threshold = 0.5f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(5);
    frequencyData << std::complex<float>(0.6f, 0.0f), std::complex<float>(0.7f, 0.1f), std::complex<float>(0.8f, -0.2f),
        std::complex<float>(0.9f, 0.3f), std::complex<float>(1.0f, -0.4f);

    EXPECT_TRUE(detector.detect(frequencyData));
}

// Test: Detection when the average magnitude is below the threshold
TEST(AverageMagnitudeDetectorTest, DoesNotDetectWhenBelowThreshold)
{
    float threshold = 1.0f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(5);
    frequencyData << std::complex<float>(0.2f, 0.0f), std::complex<float>(0.3f, 0.1f), std::complex<float>(0.4f, -0.2f),
        std::complex<float>(0.5f, 0.3f), std::complex<float>(0.6f, -0.4f);

    EXPECT_FALSE(detector.detect(frequencyData));
}

// Test: Detection when the average magnitude is exactly at the threshold
TEST(AverageMagnitudeDetectorTest, DetectsWhenAtThreshold)
{
    float threshold = 0.5f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(4);
    frequencyData << std::complex<float>(0.5f, 0.0f), std::complex<float>(0.5f, 0.0f), std::complex<float>(0.5f, 0.0f),
        std::complex<float>(0.5f, 0.0f);

    EXPECT_TRUE(detector.detect(frequencyData));
}

// Test: Detection with empty input should return false (edge case)
TEST(AverageMagnitudeDetectorTest, DoesNotDetectWithEmptyInput)
{
    float threshold = 0.5f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(0);  // Empty vector

    EXPECT_FALSE(detector.detect(frequencyData));
}

// Test: Detection with a single frequency component
TEST(AverageMagnitudeDetectorTest, SingleFrequencyComponent)
{
    float threshold = 0.7f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(1);
    frequencyData << std::complex<float>(0.8f, 0.0f);  // Magnitude is 0.8

    EXPECT_TRUE(detector.detect(frequencyData));
}

// Test: Detection with a single frequency component below threshold
TEST(AverageMagnitudeDetectorTest, SingleFrequencyComponentBelowThreshold)
{
    float threshold = 1.0f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(1);
    frequencyData << std::complex<float>(0.5f, 0.0f);  // Magnitude is 0.5

    EXPECT_FALSE(detector.detect(frequencyData));
}

// Test: Detection with complex numbers, ensuring magnitude calculation is correct
TEST(AverageMagnitudeDetectorTest, ComplexMagnitudeComputation)
{
    float threshold = 1.0f;
    AverageMagnitudeDetector detector(threshold);

    Eigen::VectorXcf frequencyData(3);
    frequencyData << std::complex<float>(0.6f, 0.8f),  // |0.6 + 0.8i| = 1.0
        std::complex<float>(0.3f, 0.4f),  // |0.3 + 0.4i| = 0.5
        std::complex<float>(0.0f, 1.0f);  // |0.0 + 1.0i| = 1.0

    float expectedAvgMag = (1.0f + 0.5f + 1.0f) / 3;
    EXPECT_EQ(detector.detect(frequencyData), expectedAvgMag >= threshold);
}
