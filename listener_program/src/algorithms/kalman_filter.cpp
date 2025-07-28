#include "kalman_filter.h"

/**
 * @brief Constructs a Kalman Filter with an initial state.
 *
 * @param initialState Initial state vector for the filter.
 */
KalmanFilter::KalmanFilter(const Eigen::Vector3f& initialState)
{
    // State transition matrix (6x6): Describes how the state evolves
    mProcessModel.resize(6, 6);
    mProcessModel << 1, 0, 0, 1, 0, 0,  // X position update
        0, 1, 0, 0, 1, 0,  // Y position update
        0, 0, 1, 0, 0, 1,  // Z position update
        0, 0, 0, 1, 0, 0,  // X velocity remains the same
        0, 0, 0, 0, 1, 0,  // Y velocity remains the same
        0, 0, 0, 0, 0, 1;  // Z velocity remains the same

    // Observation model matrix (3x6): We can observe position but not velocity
    mObservationModel.resize(3, 6);
    mObservationModel << 1, 0, 0, 0, 0, 0,  // Observing X position
        0, 1, 0, 0, 0, 0,  // Observing Y position
        0, 0, 1, 0, 0, 0;  // Observing Z position

    // Initial state vector (6x1): [X, Y, Z, Vx, Vy, Vz]
    mCurrentState.resize(6);
    mCurrentState << initialState(0), initialState(1), initialState(2), 0.0f, 0.0f, 0.0f;

    mStateCovariance.resize(6, 6);
    mStateCovariance.setIdentity();
    mStateCovariance *= 1000.0f;

    mProcessNoiseCov.resize(6, 6);
    mProcessNoiseCov.setIdentity();
    mProcessNoiseCov *= 0.01f;

    mObservationNoiseCov.resize(3, 3);
    mObservationNoiseCov.setIdentity();
    mObservationNoiseCov *= 10.0f;

    mPredictedState = mCurrentState;
    mPredictedCovariance = mStateCovariance;

    mPrevKalmanPredictTime = std::chrono::steady_clock::now();
}

void KalmanFilter::updateProcessModel(float deltaT)
{
    mProcessModel = Eigen::MatrixXf::Zero(6, 6);
    mProcessModel.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    mProcessModel.block<3, 3>(0, 3) = deltaT * Eigen::Matrix3f::Identity();
    mProcessModel.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();
}

/**
 * @brief Predicts the next state using the process model.
 */
void KalmanFilter::predict(std::chrono::time_point<std::chrono::steady_clock> currentTime)
{
    // float deltaT = (currentTime - mPrevKalmanPredictTime).count();  // in seconds
    float deltaT = std::chrono::duration<float>(currentTime - mPrevKalmanPredictTime).count();  // deltaT in seconds
    updateProcessModel(deltaT);  // Rebuild process model with latest deltaT
    mPrevKalmanPredictTime = currentTime;
    mPredictedState = mProcessModel * mCurrentState;
    mPredictedCovariance = mProcessModel * mStateCovariance * mProcessModel.transpose() + mProcessNoiseCov;
}

/**
 * @brief Updates the state estimate using an observation.
 *
 * @param observation Vector containing the observed measurements.
 */
void KalmanFilter::update(const Eigen::VectorXf& observation)
{
    Eigen::VectorXf innovation = observation - mObservationModel * mPredictedState;

    Eigen::MatrixXf innovationCovariance =
        mObservationModel * mPredictedCovariance * mObservationModel.transpose() + mObservationNoiseCov;

    Eigen::MatrixXf kalmanGain = mPredictedCovariance * mObservationModel.transpose() * innovationCovariance.inverse();

    mCurrentState = mPredictedState + kalmanGain * innovation;
    mStateCovariance = (Eigen::MatrixXf::Identity(mPredictedCovariance.rows(), mPredictedCovariance.cols()) -
                        kalmanGain * mObservationModel) *
                       mPredictedCovariance;
}

/**
 * @brief Retrieves the observation model matrix.
 *
 * @return Constant reference to the observation model matrix.
 */
const Eigen::MatrixXf& KalmanFilter::getObservationMatrix() const { return mObservationModel; }

/**
 * @brief Retrieves the predicted state vector prior to the update step.
 *
 * @return Constant reference to the predicted state vector.
 */
const Eigen::VectorXf& KalmanFilter::getPredictedState() const { return mPredictedState; }

/**
 * @brief Retrieves the current state vector after the update step.
 *
 * @return Constant reference to the current state vector.
 */
const Eigen::VectorXf& KalmanFilter::getCurrentState() const { return mCurrentState; }

/**
 * @brief Performs a full filter update, including prediction and optional observation updates.
 *
 * @param deltaT time step
 * @param filteredStateMean Optional pointer to a filtered state mean vector.
 * @param filteredStateCovariance Optional pointer to a filtered state covariance matrix.
 * @param observation Optional pointer to an observation vector.
 * @return Pair containing the updated state vector and covariance matrix.
 */

/*
auto KalmanFilter::filterUpdate(
   std::chrono::time_point<std::chrono::steady_clock> currentTime, const Eigen::VectorXf* filteredStateMean,
   const Eigen::MatrixXf* filteredStateCovariance, const Eigen::VectorXf* observation)
   -> std::pair<Eigen::VectorXf, Eigen::MatrixXf>
{
   if (filteredStateMean != nullptr) mCurrentState = *filteredStateMean;

   if (filteredStateCovariance != nullptr) mStateCovariance = *filteredStateCovariance;

   float deltaT = (currentTime - mPrevKalmanPredictTime).count();  // in seconds
   predict(deltaT);
   mPrevKalmanPredictTime = currentTime;

   if (observation != nullptr)
       update(*observation);
   else
   {
       mCurrentState = mPredictedState;
       mStateCovariance = mPredictedCovariance;
   }

   return {mCurrentState, mStateCovariance};
}
   */
