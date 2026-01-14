#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

struct DistortionMapResult {
  cv::Mat map_x;
  cv::Mat map_y;
  float min_x;
  float min_y;
  float max_x;
  float max_y;
};

class DistortionUtils {
 public:
  static std::vector<cv::Point2f> batchApplyDistortion(const std::vector<cv::Point2f>& points, const cv::Mat& K,
                                                       const cv::Mat& distCoeffs);

  static std::tuple<std::vector<cv::Point2f>, float, float, float, float> calculateBoundaryPoints(
      int W, int H, const cv::Mat& K, const cv::Mat& distCoeffs, int step = 10);

  static DistortionMapResult calculateDistortionMap(int img_w, int img_h, const cv::Mat& K, const cv::Mat& distCoeffs,
                                                    float preset_minx = std::numeric_limits<float>::quiet_NaN(),
                                                    float preset_miny = std::numeric_limits<float>::quiet_NaN(),
                                                    float preset_maxx = std::numeric_limits<float>::quiet_NaN(),
                                                    float preset_maxy = std::numeric_limits<float>::quiet_NaN());
};