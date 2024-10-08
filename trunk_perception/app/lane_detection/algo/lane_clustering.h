/**
 * @file lane_clustering.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线聚类, 从视觉SDK中移植过来的
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_algo {

class LaneClustering {
 public:
  LaneClustering(int min_points = 3, float distance_threshold = 5.0f)
      : min_points_(min_points), distance_threshold_(distance_threshold){};

  std::vector<cv::Point3f> Cluster(const std::vector<cv::Point3f>& points);

 private:
  float Distance(const cv::Point3f& pt1, const cv::Point3f& pt2);
  std::vector<int> DBSCAN(const std::vector<cv::Point3f>& points);
  std::vector<cv::Point3f> ExtractLargestCluster(const std::vector<cv::Point3f>& points,
                                                 const std::vector<int>& labels);

  int min_points_;
  float distance_threshold_;
};
}  // namespace ld_algo

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
