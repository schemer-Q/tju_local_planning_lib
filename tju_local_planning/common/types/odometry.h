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

#include <Eigen/Geometry>
#include <memory>
#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

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

  double wheel_speed = 0.0;  ///< 补偿后的车体坐标系下的纵向速度
  double speed_scale = 0.0;  ///< 定位发布的车体下的速度与vehicle_info2中的wheel_speed的比值

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
    wheel_speed = 0.0;
    speed_scale = 0.0;
  }

  typedef std::shared_ptr<Odometry> Ptr;
  typedef std::shared_ptr<const Odometry> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
