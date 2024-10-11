/**
 * @file standard_camera_projection.hpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 通过设置相机内参和外参，提供一系列相机的投影变换工具
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Dense>

#include "trunk_perception/common/types/camera_info.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

class StandardCameraProjection {
 public:
  StandardCameraProjection(const CameraInfo& camera_info, const Eigen::Isometry3f& extrinsic) {
    Init(camera_info, extrinsic);
  }
  ~StandardCameraProjection() = default;

  void Init(const CameraInfo& camera_info, const Eigen::Isometry3f& extrinsic) {
    // 初始化内参矩阵
    K_matrix_ << camera_info.fx, 0, camera_info.cx, 0, camera_info.fy, camera_info.cy, 0, 0, 1;
    camera_info_ = camera_info;

    // 初始化外参矩阵
    extrinsic_ = extrinsic;

    Eigen::Quaternionf quaternion = Eigen::Quaternionf(extrinsic_.rotation());

    r_matrix_camera_to_world_ = Eigen::Matrix4f::Identity();
    r_matrix_camera_to_world_.block<3, 3>(0, 0) = quaternion.matrix();
    r_matrix_world_to_camera_ = r_matrix_camera_to_world_.inverse();
  }

  cv::Point2f WorldToImg(const cv::Point3f& world_loc, float yaw_angle_delta = 0) {
    cv::Point2f img_point;

    // world to camera
    Eigen::Vector4f pt_loc;
    pt_loc << world_loc.x - extrinsic_.translation().x(), world_loc.y - extrinsic_.translation().y(),
        world_loc.z - extrinsic_.translation().z(), 1;
    pt_loc = (r_matrix_world_to_camera_ * pt_loc);
    Eigen::Vector3f camera_loc;
    camera_loc << pt_loc[0], pt_loc[1], pt_loc[2];

    // camera to image
    Eigen::Vector3f img_loc = (K_matrix_ * camera_loc);
    img_point.x = img_loc[0] / img_loc[2];
    img_point.y = img_loc[1] / img_loc[2];
    return img_point;
  }

  bool IsPointInView(const cv::Point2f& img_point) const {
    return img_point.x >= 0 && img_point.x < camera_info_.IMG_W && img_point.y >= 0 && img_point.y < camera_info_.IMG_H;
  }

 private:
  Eigen::Matrix3f K_matrix_;
  CameraInfo camera_info_;
  Eigen::Isometry3f extrinsic_;
  Eigen::Matrix4f r_matrix_world_to_camera_;
  Eigen::Matrix4f r_matrix_camera_to_world_;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END