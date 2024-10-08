/**
 * @file Ransac.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 从视觉SDK中移植过来的Ransac算法
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>
#include <ctime>
#include <vector>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

struct RansacResult {
  Eigen::VectorXd coefficients;
  std::vector<int> inlier_idxs;
};

class RansacPolynomialFitter {
 public:
  RansacPolynomialFitter(int maxIterations, float inlierPtsRate, int degree)
      : maxIterations_(maxIterations), inlierPtsRate_(inlierPtsRate), degree_(degree) {
    std::srand(std::time(nullptr));
  };

  Eigen::VectorXd Fit(const std::vector<Eigen::Vector2d>& points);
  bool FitWithInliers(const std::vector<Eigen::Vector2d>& points, RansacResult& result);

 private:
  std::vector<Eigen::Vector2d> RandomSample(const std::vector<Eigen::Vector2d>& points, int sampleSize);
  Eigen::VectorXd FitPolynomial(const std::vector<Eigen::Vector2d>& points);
  double CalculateError(const Eigen::VectorXd& coeffs, const Eigen::Vector2d& point);
  int CountInliers(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& model, double threshold = 1.0);
  void GetInliers(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& model, double threshold,
                  std::vector<Eigen::Vector2d>& inliers, std::vector<int>& inlier_idxs);

  int maxIterations_;
  float inlierPtsRate_;
  int degree_;
};
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END