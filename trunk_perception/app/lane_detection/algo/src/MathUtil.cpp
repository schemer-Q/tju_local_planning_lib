#include "trunk_perception/app/lane_detection/algo/MathUtil.h"
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
}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END