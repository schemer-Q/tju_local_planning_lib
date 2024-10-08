/**
 * @file camera_undistort.hpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 相机去畸变
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/camera_info.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

class CameraUndistort {
 public:
  CameraUndistort(const CameraInfo& camera_info, const bool& use_new_camera_matrix = true) {
    camera_matrix_ = camera_info.K;
    camera_distro_coeff_ = camera_info.D;
    cv::Size camera_size = cv::Size(camera_info.IMG_W, camera_info.IMG_H);

    if (use_new_camera_matrix) {
      new_camera_matrix_ =
          cv::getOptimalNewCameraMatrix(camera_matrix_, camera_distro_coeff_, camera_size, 0, camera_size, 0);
    } else {
      new_camera_matrix_ = camera_matrix_;
    }

    cv::initUndistortRectifyMap(camera_matrix_, camera_distro_coeff_, cv::Mat(), new_camera_matrix_, camera_size,
                                CV_32FC1, map_x_, map_y_);
  }
  ~CameraUndistort() = default;

  void UndistortImg(const cv::Mat& src, cv::Mat& dst) { cv::remap(src, dst, map_x_, map_y_, cv::INTER_LINEAR); }

  cv::Point2f DistortPoint(const cv::Point2f& undist_point) {
    std::vector<cv::Point2f> input_points{undist_point};
    std::vector<cv::Point2f> output_points = DistortPoints(input_points);
    return output_points[0];
  }

  std::vector<cv::Point2f> DistortPoints(const std::vector<cv::Point2f>& undist_points) {
    std::vector<cv::Point2f> dist_points;
    if (undist_points.empty()) {
      return dist_points;
    }
    const float& fx = new_camera_matrix_.at<float>(0, 0);
    const float& fy = new_camera_matrix_.at<float>(1, 1);
    const float& cx = new_camera_matrix_.at<float>(0, 2);
    const float& cy = new_camera_matrix_.at<float>(1, 2);
    const float& k1 = camera_distro_coeff_.at<float>(0);
    const float& k2 = camera_distro_coeff_.at<float>(1);
    const float& p1 = camera_distro_coeff_.at<float>(2);
    const float& p2 = camera_distro_coeff_.at<float>(3);
    const float& k3 = camera_distro_coeff_.at<float>(4);

    for (const auto& undist_point : undist_points) {
      double x = (undist_point.x - cx) / fx;
      double y = (undist_point.y - cy) / fy;

      double r2 = x * x + y * y;

      // Radial distorsion
      double xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      double yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      // Tangential distorsion
      xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
      yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

      // Back to absolute coordinates.
      xDistort = xDistort * fx + cx;
      yDistort = yDistort * fy + cy;

      dist_points.emplace_back((float)xDistort, (float)yDistort);
    }

    return dist_points;
  }

 private:
  cv::Mat camera_matrix_;        ///< 相机内参
  cv::Mat camera_distro_coeff_;  ///< 相机畸变系数
  cv::Mat new_camera_matrix_;    ///< 新的相机内参
  cv::Mat map_x_;                ///< 去畸变映射
  cv::Mat map_y_;                ///< 去畸变映射
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
