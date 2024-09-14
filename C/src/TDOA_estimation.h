/*

This file contains all function prototypes for TDOA_estimation.cpp

*/

#pragma once

#include "custom_types.h"

auto GCC_PHAT(Eigen::MatrixXcf &savedFFTs, fftwf_plan &inverseFFT,
                   const int interp, int &paddedLength,
                   const int &NUM_CHAN, const int &SAMPLE_RATE)
    -> std::tuple<Eigen::VectorXf, Eigen::VectorXf>;
Eigen::VectorXf TDOA_To_DOA_VerticalArray(Eigen::VectorXf &TDOAs, const float &soundSpeed, 
                std::span<float> chanSpacing);
Eigen::VectorXf TDOA_To_DOA_GeneralArray(const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> &qr, const float c, const Eigen::VectorXf &tdoa);
//Eigen::VectorXf CrossCorr(const Eigen::MatrixXf &channel_matrix, float fs, float max_tau, int interp);
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> precomputedQR(const Eigen::MatrixXd &H);