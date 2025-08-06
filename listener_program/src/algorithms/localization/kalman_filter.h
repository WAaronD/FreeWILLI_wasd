#pragma once
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
/**
 * @class KalmanFilter
 * @brief Implements a Kalman Filter for linear state estimation. This includes
 * prediction, observation updates, and state retrieval for real-time applications.
 */
class KalmanFilter
{
   public:
    explicit KalmanFilter(const Eigen::Vector3f& initialState);
    void predict(std::chrono::time_point<std::chrono::steady_clock> currentTime);  // Now accepts deltaT as input
    void update(const Eigen::VectorXf& observation);
    /*
    auto filterUpdate(
        std::chrono::time_point<std::chrono::steady_clock> currentTime,
        const Eigen::VectorXf* filteredStateMean = nullptr, const Eigen::MatrixXf* filteredStateCovariance = nullptr,
        const Eigen::VectorXf* observation = nullptr) -> std::pair<Eigen::VectorXf, Eigen::MatrixXf>;
    */
    const Eigen::MatrixXf& getObservationMatrix() const;
    const Eigen::VectorXf& getPredictedState() const;
    const Eigen::VectorXf& getCurrentState() const;

   private:
    void updateProcessModel(float deltaT);  // Helper function to rebuild F_k with new deltaT

    Eigen::MatrixXf mProcessModel;  ///< Process model matrix (state transition matrix).
    Eigen::MatrixXf mObservationModel;  ///< Observation model matrix.
    Eigen::MatrixXf mProcessNoiseCov;  ///< Process noise covariance matrix.
    Eigen::MatrixXf mObservationNoiseCov;  ///< Observation noise covariance matrix.
    Eigen::VectorXf mCurrentState;  ///< Current state vector.
    Eigen::MatrixXf mStateCovariance;  ///< State covariance matrix.
    Eigen::VectorXf mPredictedState;  ///< Predicted state vector (prior to update).
    Eigen::MatrixXf mPredictedCovariance;  ///< Predicted state covariance matrix (prior to update).

    std::chrono::time_point<std::chrono::steady_clock> mPrevKalmanPredictTime = std::chrono::steady_clock::now();
};