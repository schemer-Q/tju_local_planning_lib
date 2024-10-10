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
 * @param polygon1 多变型1
 * @param polygon2 多变型2
 * @param scale 分辨率
 * @return double 重叠率
 */
double getOverlapRate(const Eigen::MatrixXf& polygon1, const Eigen::MatrixXf& polygon2, const int& scale = 10);

TRUNK_PERCEPTION_LIB_NAMESPACE_END