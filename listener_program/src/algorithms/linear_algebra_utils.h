#pragma once

#include <eigen3/Eigen/Dense>

auto computeSvd(const Eigen::MatrixXf& H) -> Eigen::JacobiSVD<Eigen::MatrixXf>;
Eigen::MatrixXf precomputePseudoInverse(const Eigen::JacobiSVD<Eigen::MatrixXf>& svd);
int computeRank(const Eigen::MatrixXf& H, double tolerance);
