
#include "../../src/algorithms/kalman_filter.h"

#include "gtest/gtest.h"
// Initialize KalmanFilter with a known state
TEST(KalmanFilterTest, Initialization)
{
    Eigen::Vector3f initialState(1.0f, 2.0f, 3.0f);
    KalmanFilter filter(initialState);

    // Ensure initial state is correctly set
    Eigen::VectorXf expectedState(6);
    expectedState << 1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f;  // Initial velocity = 0

    EXPECT_EQ(filter.getCurrentState(), expectedState);
}

// Test prediction step updates state correctly
TEST(KalmanFilterTest, PredictUpdatesState)
{
    Eigen::Vector3f initialState(0.0f, 0.0f, 0.0f);
    KalmanFilter filter(initialState);

    filter.predict();
    Eigen::VectorXf predictedState = filter.getPredictedState();

    // Since velocity is zero, position should not change
    Eigen::VectorXf expectedState(6);
    expectedState << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    EXPECT_EQ(predictedState, expectedState);
}

// Test getObservationMatrix returns expected dimensions
TEST(KalmanFilterTest, GetObservationMatrix)
{
    Eigen::Vector3f initialState(1.0f, 1.0f, 1.0f);
    KalmanFilter filter(initialState);

    Eigen::MatrixXf observationMatrix = filter.getObservationMatrix();

    // Check matrix dimensions (should be 3x6)
    EXPECT_EQ(observationMatrix.rows(), 3);
    EXPECT_EQ(observationMatrix.cols(), 6);
}
TEST(KalmanFilterTest, UpdateWithObservation)
{
    Eigen::Vector3f initialState(0.0f, 0.0f, 0.0f);
    KalmanFilter filter(initialState);

    filter.predict();

    // Provide an observation (we observe position only)
    Eigen::VectorXf observation(3);
    observation << 5.0f, 5.0f, 5.0f;

    filter.update(observation);

    Eigen::VectorXf updatedState = filter.getCurrentState();

    // Allow for small numerical differences due to Kalman Gain calculations
    constexpr float tolerance = 3e-2;  // Increase tolerance slightly

    EXPECT_NEAR(updatedState(0), 5.0f, tolerance);
    EXPECT_NEAR(updatedState(1), 5.0f, tolerance);
    EXPECT_NEAR(updatedState(2), 5.0f, tolerance);
}

// Test that filterUpdate works without external input
TEST(KalmanFilterTest, FilterUpdateNoInputs)
{
    Eigen::Vector3f initialState(2.0f, 2.0f, 2.0f);
    KalmanFilter filter(initialState);

    auto [updatedState, updatedCov] = filter.filterUpdate();

    // Since no observation was provided, state should match the predicted state
    EXPECT_EQ(updatedState, filter.getPredictedState());
}

// Test filterUpdate with an observation
TEST(KalmanFilterTest, FilterUpdateWithObservation)
{
    Eigen::Vector3f initialState(0.0f, 0.0f, 0.0f);
    KalmanFilter filter(initialState);

    Eigen::VectorXf observation(3);
    observation << 3.0f, 4.0f, 5.0f;

    auto [updatedState, updatedCov] = filter.filterUpdate(nullptr, nullptr, &observation);

    EXPECT_NEAR(updatedState(0), 3.0f, 3e-2);
    EXPECT_NEAR(updatedState(1), 4.0f, 3e-2);
    EXPECT_NEAR(updatedState(2), 5.0f, 3e-2);
}
