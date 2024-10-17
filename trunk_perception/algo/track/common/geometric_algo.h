/**
 * @file geometric_algo.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

using common::BoundingBox;
using common::LShapeFeature;
using common::TailCenterFeature;

constexpr float RAD2DEG = 180.0F / M_PI;
constexpr float DEG2RAD = M_PI / 180.0F;

/**
 * @brief 计算Lshape特征
 *
 * @param bbox BoundingBox信息
 * @param shape_feature Lshape特征信息
 */
void computeLShapeFeature(const BoundingBox& bbox, LShapeFeature& shape_feature);

/**
 * @brief 计算两个多边形之间的重叠率
 *
 * @param polygon1 多边形1
 * @param polygon2 多边形2
 * @param scale 分辨率
 * @return double 重叠率
 */
double getOverlapRate(const Eigen::MatrixXf& polygon1, const Eigen::MatrixXf& polygon2, const int& scale = 10);

/**
 * @brief 计算两个角度之间的差值
 *
 * @tparam T 数据类型
 * @param from_angle 源角度[-pi,pi]
 * @param to_angle 目标角度[-pi,pi]
 * @return T 角度差值[-pi,pi]
 */
template <typename T>
T getAngleDiff(const T from_angle, const T to_angle) {
  const T angle_diff = from_angle - to_angle;
  if (-M_PI <= angle_diff && angle_diff <= M_PI) {
    return angle_diff;
  } else if (angle_diff > M_PI) {
    return angle_diff - 2 * M_PI;
  } else if (angle_diff < -M_PI) {
    return angle_diff + 2 * M_PI;
  } else {
    return static_cast<T>(0.0);
  }
}

/**
 * @brief 计算尾边中心点特征
 *
 * @param bbox BoundingBox信息
 * @param feature 尾边中心点特征
 */
void computeTailCenterFeature(const BoundingBox& bbox, TailCenterFeature& feature);

TRUNK_PERCEPTION_LIB_NAMESPACE_END