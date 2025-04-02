#pragma once

#include "../pch.h"
void normalizeDoa(Eigen::VectorXf& doaVector, int solutionRank);
auto computeDoaFromTdoa(const Eigen::MatrixXf& cachedLeastSquaresResult, const Eigen::VectorXf& tdoa, int rank)
    -> Eigen::VectorXf;

auto convertDoaToElAz(const Eigen::VectorXf& doa) -> Eigen::VectorXf;
