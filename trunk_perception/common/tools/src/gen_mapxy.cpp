#include "../gen_mapxy.h"

std::vector<cv::Point2f> DistortionUtils::batchApplyDistortion(const std::vector<cv::Point2f>& points, const cv::Mat& K,
                                                               const cv::Mat& distCoeffs) {
  float fx = K.at<double>(0, 0);
  float fy = K.at<double>(1, 1);
  float cx = K.at<double>(0, 2);
  float cy = K.at<double>(1, 2);

  std::vector<cv::Point2f> distorted_points(points.size());
  std::vector<cv::Point2f> normalized_points(points.size());

// 1. 首先将像素坐标转换为归一化坐标
#pragma omp simd
  for (size_t i = 0; i < points.size(); i++) {
    normalized_points[i].x = (points[i].x - cx) / fx;
    normalized_points[i].y = (points[i].y - cy) / fy;
  }

  bool is_fisheye = (distCoeffs.total() == 4);
  bool is_8param = (distCoeffs.total() == 8);

  if (is_fisheye) {
    // 鱼眼相机畸变模型
    const float k1 = distCoeffs.at<double>(0);
    const float k2 = distCoeffs.at<double>(1);
    const float k3 = distCoeffs.at<double>(2);
    const float k4 = distCoeffs.at<double>(3);

#pragma omp parallel for
    for (size_t i = 0; i < normalized_points.size(); i++) {
      float x = normalized_points[i].x;
      float y = normalized_points[i].y;
      float r = std::sqrt(x * x + y * y);

      if (r > 1e-8) {
        float theta = std::atan(r);
        float theta_d = theta * (1 + k1 * theta * theta + k2 * theta * theta * theta * theta +
                                 k3 * theta * theta * theta * theta * theta * theta +
                                 k4 * theta * theta * theta * theta * theta * theta * theta * theta);
        float scale = (r > 0) ? (theta_d / r) : 1.0;

        distorted_points[i].x = fx * (x * scale) + cx;
        distorted_points[i].y = fy * (y * scale) + cy;
      } else {
        distorted_points[i] = points[i];
      }
    }
  } else if (is_8param) {
    // 8参数畸变模型
    const float k1 = distCoeffs.at<double>(0);
    const float k2 = distCoeffs.at<double>(1);
    const float p1 = distCoeffs.at<double>(2);
    const float p2 = distCoeffs.at<double>(3);
    const float k3 = distCoeffs.at<double>(4);
    const float k4 = distCoeffs.at<double>(5);
    const float k5 = distCoeffs.at<double>(6);
    const float k6 = distCoeffs.at<double>(7);

#pragma omp parallel for
    for (size_t i = 0; i < normalized_points.size(); i++) {
      float x = normalized_points[i].x;
      float y = normalized_points[i].y;
      float r2 = x * x + y * y;
      float r4 = r2 * r2;
      float r6 = r4 * r2;

      // 计算径向畸变（增加了k4, k5, k6项）
      float radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r2 * r6 + k5 * r4 * r6 + k6 * r6 * r6;

      // 计算切向畸变
      float x_distorted = x * radial_distortion + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      float y_distorted = y * radial_distortion + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

      // 转回像素坐标
      distorted_points[i].x = fx * x_distorted + cx;
      distorted_points[i].y = fy * y_distorted + cy;
    }
  } else {
    // 标准相机畸变模型
    const float k1 = distCoeffs.at<double>(0);
    const float k2 = distCoeffs.at<double>(1);
    const float p1 = distCoeffs.at<double>(2);
    const float p2 = distCoeffs.at<double>(3);
    const float k3 = distCoeffs.at<double>(4);

#pragma omp parallel for
    for (size_t i = 0; i < normalized_points.size(); i++) {
      float x = normalized_points[i].x;
      float y = normalized_points[i].y;
      float r2 = x * x + y * y;
      float r4 = r2 * r2;
      float r6 = r4 * r2;

      // 计算径向畸变
      float radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6;

      // 计算切向畸变
      float x_distorted = x * radial_distortion + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      float y_distorted = y * radial_distortion + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

      // 转回像素坐标
      distorted_points[i].x = fx * x_distorted + cx;
      distorted_points[i].y = fy * y_distorted + cy;
    }
  }

  return distorted_points;
}

std::tuple<std::vector<cv::Point2f>, float, float, float, float> DistortionUtils::calculateBoundaryPoints(
    int W, int H, const cv::Mat& K, const cv::Mat& distCoeffs, int step) {
  // 1. 预计算边界点数量并预分配内存
  int num_points = 2 * ((W / step) + (H / step)) + 4;  // 估算点数
  std::vector<cv::Point2f> distorted_points;
  distorted_points.reserve(num_points);

  // 2. 使用更高效的点生成方式
  {  // 上边界
    float y = 0;
    for (int x = 0; x < W - step; x += step) {
      distorted_points.emplace_back(x, y);
    }
  }
  {  // 右边界
    float x = W - 1;
    for (int y = 0; y < H - step; y += step) {
      distorted_points.emplace_back(x, y);
    }
  }
  {  // 下边界
    float y = H - 1;
    for (int x = W - 1; x > step; x -= step) {
      distorted_points.emplace_back(x, y);
    }
  }
  {  // 左边界
    float x = 0;
    for (int y = H - 1; y > step; y -= step) {
      distorted_points.emplace_back(x, y);
    }
  }

  // 3. 预分配去畸变点的内存
  std::vector<cv::Point2f> undistorted_points;
  undistorted_points.reserve(distorted_points.size());

  // 4. 提前获取相机参数避免重复访问
  const float fx = K.at<double>(0, 0);
  const float fy = K.at<double>(1, 1);
  const float cx = K.at<double>(0, 2);
  const float cy = K.at<double>(1, 2);

  // 5. 计算去畸变点
  if (distCoeffs.total() == 4) {
    cv::fisheye::undistortPoints(distorted_points, undistorted_points, K, distCoeffs);
  } else {
    cv::undistortPoints(distorted_points, undistorted_points, K, distCoeffs);
  }

  // 6. 使用SIMD优化的方式处理点坐标转换和边界计算
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();

#pragma omp simd reduction(min : min_x, min_y) reduction(max : max_x, max_y)
  for (size_t i = 0; i < undistorted_points.size(); ++i) {
    // 转回像素坐标
    undistorted_points[i].x = undistorted_points[i].x * fx + cx;
    undistorted_points[i].y = undistorted_points[i].y * fy + cy;

    // 同时计算边界
    min_x = std::min(min_x, undistorted_points[i].x);
    min_y = std::min(min_y, undistorted_points[i].y);
    max_x = std::max(max_x, undistorted_points[i].x);
    max_y = std::max(max_y, undistorted_points[i].y);
  }

  return {std::move(undistorted_points), min_x, min_y, max_x, max_y};
}

DistortionMapResult DistortionUtils::calculateDistortionMap(int img_w, int img_h, const cv::Mat& K,
                                                            const cv::Mat& distCoeffs, float preset_minx,
                                                            float preset_miny, float preset_maxx, float preset_maxy) {
  // 1. 计算边界点
  auto [boundary_points, min_x, min_y, max_x, max_y] = calculateBoundaryPoints(img_w, img_h, K, distCoeffs, 10);

  min_x = std::isnan(preset_minx) ? min_x : preset_minx;
  min_y = std::isnan(preset_miny) ? min_y : preset_miny;
  max_x = std::isnan(preset_maxx) ? max_x : preset_maxx;
  max_y = std::isnan(preset_maxy) ? max_y : preset_maxy;

  // 2. 生成网格点并判断是否在边界内
  int grid_h = static_cast<int>(max_y - min_y);
  int grid_w = static_cast<int>(max_x - min_x);

  cv::Mat map_x = cv::Mat::zeros(grid_h, grid_w, CV_32F);
  cv::Mat map_y = cv::Mat::zeros(grid_h, grid_w, CV_32F);

  std::vector<cv::Point2f> grid_points(grid_h * grid_w);

#pragma omp parallel for collapse(2)
  for (int y = 0; y < grid_h; y++) {
    for (int x = 0; x < grid_w; x++) {
      const int idx = y * grid_w + x;
      grid_points[idx] = cv::Point2f(x + min_x, y + min_y);
    }
  }

  // 判断点是否在边界内
  std::vector<cv::Point> boundary_points_int;
  for (const auto& pt : boundary_points) {
    boundary_points_int.emplace_back(cvRound(pt.x), cvRound(pt.y));
  }

  std::vector<cv::Point2f> distorted_points = batchApplyDistortion(grid_points, K, distCoeffs);

  // 预先计算指针和步长
  float* map_x_ptr = reinterpret_cast<float*>(map_x.data);
  float* map_y_ptr = reinterpret_cast<float*>(map_y.data);
  const int stride = map_x.step1();

  // 预计算网格大小
  const int grid_size = static_cast<int>(grid_points.size());
  const float nan_value = std::numeric_limits<float>::quiet_NaN();

  // 使用查找表预计算round结果
  std::vector<cv::Point> points_round(grid_points.size());
  for (size_t i = 0; i < grid_points.size(); ++i) {
    points_round[i] = cv::Point(cvRound(grid_points[i].x), cvRound(grid_points[i].y));
  }

  // 使用分块处理来提高缓存命中率
  const int block_size = 256;  // 根据CPU缓存大小调整
#pragma omp parallel for schedule(dynamic, 1)
  for (int block = 0; block < grid_size; block += block_size) {
    const int end = std::min(block + block_size, grid_size);

    for (int i = block; i < end; ++i) {
      const int y = static_cast<int>(grid_points[i].y - min_y);
      const int x = static_cast<int>(grid_points[i].x - min_x);
      const int offset = y * stride + x;

      // 使用预计算的round结果
      const double dist = cv::pointPolygonTest(boundary_points_int, points_round[i], false);

      if (dist >= 0) {
        map_x_ptr[offset] = distorted_points[i].x;
        map_y_ptr[offset] = distorted_points[i].y;
      } else {
        map_x_ptr[offset] = nan_value;
        map_y_ptr[offset] = nan_value;
      }
    }
  }

  return {map_x, map_y, min_x, min_y, max_x, max_y};
}