#include "../../src/algorithms/hydrophone_position_processing.h"

#include "gtest/gtest.h"

// Test: Simple case with hydrophones along the Z-axis
TEST(CalculateRelativePositionsTest, HydrophonesAlongZAxis)
{
    Eigen::MatrixXf positions(4, 3);
    positions << 0, 0, 0, 0, 0, -1, 0, 0, -2, 0, 0, -3;

    Eigen::MatrixXf expected(6, 3);
    expected << 0, 0, -1, 0, 0, -2, 0, 0, -3, 0, 0, -1, 0, 0, -2, 0, 0, -1;

    Eigen::MatrixXf result = calculateRelativePositions(positions);

    EXPECT_EQ(result.rows(), expected.rows());
    EXPECT_EQ(result.cols(), expected.cols());

    for (int i = 0; i < expected.rows(); ++i)
    {
        for (int j = 0; j < expected.cols(); ++j)
        {
            EXPECT_NEAR(result(i, j), expected(i, j), 1e-6);
        }
    }
}

// Test: Arbitrary 3D positions

TEST(CalculateRelativePositionsTest, ArbitraryPositions)
{
    Eigen::MatrixXf positions(4, 3);
    positions << 0, 0, 0, -0.491908451523206, 0.352787551720565, -0.824159687290875, -0.142432531081157,
        -0.586049330403225, -0.835786971723162, 0.482502162552815, 0.182323162667854, -0.876018777939435;

    Eigen::MatrixXf expected(6, 3);
    expected << -0.491908, 0.352788, -0.82416, -0.142433, -0.586049, -0.835787, 0.482502, 0.182323, -0.876019, 0.349476,
        -0.938837, -0.0116273, 0.974411, -0.170464, -0.0518591, 0.624935, 0.768372, -0.0402318;

    Eigen::MatrixXf result = calculateRelativePositions(positions);

    EXPECT_EQ(result.rows(), expected.rows());
    EXPECT_EQ(result.cols(), expected.cols());

    for (int i = 0; i < expected.rows(); ++i)
    {
        for (int j = 0; j < expected.cols(); ++j)
        {
            EXPECT_NEAR(result(i, j), expected(i, j), 1e-6);
        }
    }
}
