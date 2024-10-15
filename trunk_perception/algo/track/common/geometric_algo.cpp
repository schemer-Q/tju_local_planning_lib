/**
 * @file geometric_algo.cpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

void computeLShapeFeature(const BoundingBox& bbox, LShapeFeature& shape_feature) {
  shape_feature.center_point = bbox.center.head(2).cast<double>();
  shape_feature.reference_point = bbox.corners2d.col(0).cast<double>();
  shape_feature.shape(0) = static_cast<double>((bbox.corners2d.col(1) - bbox.corners2d.col(0)).norm());
  shape_feature.shape(1) = static_cast<double>((bbox.corners2d.col(3) - bbox.corners2d.col(0)).norm());
  shape_feature.shape(2) = static_cast<double>(
      std::atan2((bbox.corners2d(1, 1) - bbox.corners2d(1, 0)), (bbox.corners2d(0, 1) - bbox.corners2d(0, 0))));
}

double getOverlapRate(const Eigen::MatrixXf& polygon1, const Eigen::MatrixXf& polygon2, const int& scale) {
  if (polygon1.rows() != 2 && polygon1.rows() != 3) {
    TFATAL << "polygon1 input size(rows, cols) error!";
  }

  if (polygon2.rows() != 2 && polygon2.rows() != 3) {
    TFATAL << "polygon2 input size(rows, cols) error!";
  }

  const float min_x_1 = polygon1.row(0).minCoeff();
  const float max_x_1 = polygon1.row(0).maxCoeff();
  const float min_y_1 = polygon1.row(1).minCoeff();
  const float max_y_1 = polygon1.row(1).maxCoeff();
  const float min_x_2 = polygon2.row(0).minCoeff();
  const float max_x_2 = polygon2.row(0).maxCoeff();
  const float min_y_2 = polygon2.row(1).minCoeff();
  const float max_y_2 = polygon2.row(1).maxCoeff();

  if (min_x_1 > max_x_2 || min_y_1 > max_y_2 || min_x_2 > max_x_1 || min_y_2 > max_y_1) {
    return 0.0;
  }

  const float min_x = std::min(min_x_1, min_x_2);
  const float min_y = std::min(min_y_1, min_y_2);
  const float max_x = std::max(max_x_1, max_x_2);
  const float max_y = std::max(max_y_1, max_y_2);

  const int h = static_cast<int>((max_y - min_y + 2.0F) * scale);
  const int w = static_cast<int>((max_x - min_x + 2.0F) * scale);
  cv::Mat im_1 = cv::Mat::zeros(h, w, CV_8UC1);
  cv::Mat im_2 = cv::Mat::zeros(h, w, CV_8UC1);

  const int size_1 = polygon1.cols();
  const int size_2 = polygon2.cols();
  std::vector<cv::Point> pts1(size_1);
  std::vector<cv::Point> pts2(size_2);
  for (int i = 0; i < size_1; ++i) {
    pts1[i].x = static_cast<int>((polygon1(0, i) - min_x + 1.0F) * scale);
    pts1[i].y = static_cast<int>((polygon1(1, i) - min_y + 1.0F) * scale);
  }
  for (int i = 0; i < size_2; ++i) {
    pts2[i].x = static_cast<int>((polygon2(0, i) - min_x + 1.0F) * scale);
    pts2[i].y = static_cast<int>((polygon2(1, i) - min_y + 1.0F) * scale);
  }

  cv::fillConvexPoly(im_1, pts1, cv::Scalar(1));
  cv::fillConvexPoly(im_2, pts2, cv::Scalar(1));
  cv::Mat im_inner = im_1 & im_2;

  const double inner = cv::sum(im_inner).val[0];
  const double v1 = cv::sum(im_1).val[0];
  const double v2 = cv::sum(im_2).val[0];
  return inner / (std::min(v1, v2) + 1E-6);
}

void computeTailCenterFeature(const BoundingBox& bbox, TailCenterFeature& feature) {
  const auto& corners2d = bbox.corners2d;
  const int sz = corners2d.cols();
  auto edge_center_points_tmp = corners2d;
  for (int i = 0; i < sz; ++i) {
    edge_center_points_tmp.col(i) = (corners2d.col(i) + corners2d.col((i + 1) % sz)) * 0.5F;
  }

  int nearest_id = 0;
  edge_center_points_tmp.row(0).minCoeff(&nearest_id);
  for (int i = 0; i < sz; ++i) {
    feature.edge_center_points.col(i) = edge_center_points_tmp.col((nearest_id + i) % sz).cast<double>();
  }
  feature.tail_center_point = feature.edge_center_points.col(0);
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END