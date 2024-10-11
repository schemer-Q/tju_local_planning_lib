#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
#include <Eigen/Dense>
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

// polyfitting
bool PolyFitting(const std::vector<cv::Point3f>& in_point, const int& n, cv::Mat& out) {
  std::vector<float> xs;
  std::vector<float> ys;
  for (size_t i = 0; i < in_point.size(); i++) {
    xs.push_back(in_point[i].x);
    ys.push_back(in_point[i].y);
  }
  return PolyFitting(xs, ys, n, out);
}

// todo 返回bool
bool PolyFitting(const std::vector<cv::Point2f>& in_point, const int& n, cv::Mat& out) {
  std::vector<float> xs;
  std::vector<float> ys;
  for (size_t i = 0; i < in_point.size(); i++) {
    xs.push_back(in_point[i].x);
    ys.push_back(in_point[i].y);
  }
  return PolyFitting(xs, ys, n, out);
}

bool PolyFitting(const std::vector<float>& xs, const std::vector<float>& ys, const int& n, cv::Mat& out) {
  size_t size = xs.size();
  if (ys.size() != size || size < size_t(n)) {
    TERROR << "PolyFitting: THE LANE PTS NUM IS NOT ENOUGH";
    return false;
  }
  // 所求未知数个数
  int x_num = n + 1;
  // 构造矩阵U和Y
  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j) {
      mat_u.at<double>(i, j) = pow(xs[i], j);
    }

  for (int i = 0; i < mat_y.rows; ++i) {
    mat_y.at<double>(i, 0) = ys[i];
  }

  // 矩阵运算，获得系数矩阵K
  cv::Mat mat_k = cv::Mat::zeros(x_num, 1, CV_64F);
  mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  out = mat_k;

  return true;
}

std::vector<PolynomialCoefficients> FitParallelLines(const std::vector<std::vector<std::pair<float, float>>>& points,
                                                     bool weighted, int poly_num) {
  int num_lines = points.size();
  int num_pts = 0;
  for (const auto& pts : points) {
    num_pts += pts.size();
  }

  Eigen::MatrixXd X(num_pts, num_lines + poly_num);  // Design matrix
  Eigen::VectorXd Y(num_pts);                        // Response vector
  Eigen::VectorXd weights(num_pts);                  // Weights vector

  // Fill in the design matrix
  int tmp_pt_idx = 0;
  for (size_t line_idx = 0; line_idx < points.size(); ++line_idx) {
    const auto& line_pts = points[line_idx];
    for (const auto pt : line_pts) {
      float x = pt.first;
      float y = pt.second;
      X.row(tmp_pt_idx).setZero();
      X(tmp_pt_idx, line_idx) = 1;
      float multiply_x = x;
      for (int i = 0; i < poly_num; i++) {
        X(tmp_pt_idx, num_lines + i) = multiply_x;
        multiply_x *= x;
      }

      Y(tmp_pt_idx) = y;
      if (weighted) {
        weights(tmp_pt_idx) = 1;
      } else {
        weights(tmp_pt_idx) = 0.2 * 1.0 / (1.0 + std::abs(x * x)) + 0.8 * 1.0 / (1.0 + std::abs(y * y));
      }
      tmp_pt_idx += 1;
    }
  }
  Eigen::MatrixXd W = weights.asDiagonal();
  Eigen::MatrixXd WX = W * X;
  Eigen::VectorXd WY = W * Y;
  Eigen::VectorXd result = (WX.transpose() * WX).ldlt().solve(WX.transpose() * WY);

  // get result
  std::vector<PolynomialCoefficients> results(num_lines);
  for (size_t line_idx = 0; line_idx < points.size(); ++line_idx) {
    results[line_idx].a0 = result(line_idx);
    if (poly_num >= 1) results[line_idx].a1 = result(num_lines);
    if (poly_num >= 2) results[line_idx].a2 = result(num_lines + 1);
    if (poly_num >= 3) results[line_idx].a3 = result(num_lines + 2);
  }
  return results;
}

}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END