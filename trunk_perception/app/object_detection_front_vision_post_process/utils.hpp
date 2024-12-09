/**
 * @file utils.hpp
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Eigen/Geometry>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/odometry.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

static Eigen::Isometry3f calculateTransformMatrix(const common::Odometry::ConstPtr& pose1,
                                                  const common::Odometry::ConstPtr& pose2) {
  if (!pose1 || !pose2) {
    TERROR << "[calculateTransformMatrix] input pose is nullptr!";
    return Eigen::Isometry3f::Identity();
  }

  Eigen::Matrix4d pose_diff = pose2->Matrix() - pose1->Matrix();
  if (pose_diff.norm() > 20.0) {
    TWARNING << "[calculateTransformMatrix] odometry data warning";
  }

  const Eigen::Matrix4f T12 = (pose2->Matrix().inverse() * pose1->Matrix()).cast<float>();
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.rotate(T12.block<3, 3>(0, 0));
  tf.pretranslate(T12.block<3, 1>(0, 3));
  return tf;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END