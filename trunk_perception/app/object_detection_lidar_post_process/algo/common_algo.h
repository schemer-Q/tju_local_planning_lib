/**
 * @file common_algo.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-10-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

using namespace TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE;

// 点云下采样输出OpenCV点
void downSamplePointCloud2D(const PointCloudT& pc_in, const Eigen::Vector2f& resolution,
                            std::vector<cv::Point2f>& pt_out);

// 计算polygon
int computeConvexHull(const PointCloudT& pc_in, Eigen::Matrix3Xf& polygon);

// 计算包含中心点的polygon
int computeCenterConvexHull(const PointCloudT& pc_in, const Eigen::Vector2f& center_point, Eigen::Matrix3Xf& polygon);

void computeBoundingBox(const Eigen::Matrix3Xf& polygon, const float theta, BoundingBox& bbox);

void computeBoundingBox(const PointCloudT& pc_in, const float theta, BoundingBox& bbox);

float computeFreespaceArea(const Eigen::Matrix3Xf& polygon, const BoundingBox& bbox);

void computeVisibleLine(Eigen::Matrix3Xf& polygon);

void polygon2BBox(Eigen::Matrix3Xf& polygon, BoundingBox& bbox);

double getOverlapRate(const Eigen::MatrixXf& polygon1, const Eigen::MatrixXf& polygon2, const int& scale = 10);

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END