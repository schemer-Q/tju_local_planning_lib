/**
 * @file odometry.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>
#include <Eigen/Geometry>
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 里程计数据
 * 
 */
struct Odometry {
  double time = 0.0;
  Eigen::Vector3d position;        ///< 位置
  Eigen::Quaterniond orientation;  ///< 姿态
  Eigen::Vector3d linear;          ///< 线速度
  Eigen::Vector3d angular;         ///< 角速度

  /**
   * @brief 返回位姿矩阵
   * 
   * @return Eigen::Matrix4d 
   */
  Eigen::Matrix4d Matrix() const {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = orientation.matrix();
    mat.block<3, 1>(0, 3) = position;
    return mat;
  }

  void reset() {
    position.setZero();
    orientation.setIdentity();
    linear.setZero();
    angular.setZero();
  }

  typedef std::shared_ptr<Odometry> Ptr;
  typedef std::shared_ptr<const Odometry> ConstPtr;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END