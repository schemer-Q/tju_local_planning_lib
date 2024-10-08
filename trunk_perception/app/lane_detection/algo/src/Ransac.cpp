#include "trunk_perception/app/lane_detection/algo/Ransac.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

Eigen::VectorXd RansacPolynomialFitter::Fit(const std::vector<Eigen::Vector2d>& points) {
  int bestInliers = 0;
  Eigen::VectorXd bestModel(degree_ + 1);

  for (int i = 0; i < maxIterations_; ++i) {
    std::vector<Eigen::Vector2d> sample = RandomSample(points, degree_ + 1);
    Eigen::VectorXd model = FitPolynomial(sample);

    int inliers = CountInliers(points, model);
    if (inliers > bestInliers) {
      bestInliers = inliers;
      bestModel = model;

      if (inliers >= points.size() * inlierPtsRate_) {
        break;
      }
    }
  }

  return bestModel;
}

bool RansacPolynomialFitter::FitWithInliers(const std::vector<Eigen::Vector2d>& points, RansacResult& result) {
  int bestInliers = 0;
  Eigen::VectorXd bestModel(degree_ + 1);
  std::vector<Eigen::Vector2d> bestInlierPoints;
  std::vector<int> bestInlierIdxs;

  for (int i = 0; i < maxIterations_; ++i) {
    std::vector<Eigen::Vector2d> sample = RandomSample(points, degree_ + 1);
    Eigen::VectorXd model = FitPolynomial(sample);

    std::vector<Eigen::Vector2d> inlierPoints;
    std::vector<int> inlierIdxs;
    GetInliers(points, model, 1.0, inlierPoints, inlierIdxs);
    int inliers = inlierPoints.size();
    if (inliers > bestInliers) {
      bestInliers = inliers;
      bestModel = model;
      bestInlierPoints = inlierPoints;
      bestInlierIdxs = inlierIdxs;

      if (inliers >= points.size() * inlierPtsRate_) {
        result.coefficients = bestModel;
        result.inlier_idxs = bestInlierIdxs;
        return true;
      }
    }
  }
  return false;
}

std::vector<Eigen::Vector2d> RansacPolynomialFitter::RandomSample(const std::vector<Eigen::Vector2d>& points,
                                                                  int sampleSize) {
  std::vector<Eigen::Vector2d> sample;
  while (sample.size() < size_t(sampleSize)) {
    int idx = std::rand() % points.size();
    sample.push_back(points[idx]);
  }
  return sample;
}

Eigen::VectorXd RansacPolynomialFitter::FitPolynomial(const std::vector<Eigen::Vector2d>& points) {
  Eigen::MatrixXd A(points.size(), degree_ + 1);
  Eigen::VectorXd b(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    for (int j = 0; j <= degree_; ++j) {
      A(i, j) = std::pow(points[i].x(), j);
    }
    b(i) = points[i].y();
  }

  return (A.transpose() * A).ldlt().solve(A.transpose() * b);
}

double RansacPolynomialFitter::CalculateError(const Eigen::VectorXd& coeffs, const Eigen::Vector2d& point) {
  double y_pred = 0;
  for (int i = 0; i <= degree_; ++i) {
    y_pred += coeffs(i) * std::pow(point.x(), i);
  }
  return std::abs(point.y() - y_pred);
}

int RansacPolynomialFitter::CountInliers(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& model,
                                         double threshold) {
  int inliers = 0;
  for (const auto& point : points) {
    if (CalculateError(model, point) < threshold) {
      inliers++;
    }
  }
  return inliers;
}

void RansacPolynomialFitter::GetInliers(const std::vector<Eigen::Vector2d>& points, const Eigen::VectorXd& model,
                                        double threshold, std::vector<Eigen::Vector2d>& inliers,
                                        std::vector<int>& inlier_idxs) {
  for (size_t i = 0; i < points.size(); i++) {
    auto& pt = points[i];
    if (CalculateError(model, pt) < threshold) {
      inliers.push_back(pt);
      inlier_idxs.push_back(i);
    }
  }
}
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
