/**
 * @file shape_fusion_lidar_only_impl.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于激光雷达的形状融合实现
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <deque>

#include "shape_fusion_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

struct ShapeFusionLidarOnlyConfig : public ShapeFusionConfig {
  int window_size;
  int min_valid_frame;
  float threshold;
  
  typedef std::shared_ptr<ShapeFusionLidarOnlyConfig> Ptr;
  typedef std::shared_ptr<const ShapeFusionLidarOnlyConfig> ConstPtr;
};

/**
 * @brief 基于滑动窗口的形状融合实现
 *
 */
class ShapeFusionLidarOnlyImpl : public ShapeFusionBase {
 public:
  ShapeFusionLidarOnlyImpl(const ShapeFusionConfig::ConstPtr& config);
  ~ShapeFusionLidarOnlyImpl() = default;

  std::uint32_t Init(const Eigen::Vector3f& size) override;

  std::uint32_t Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) override;

 private:
  Eigen::Vector3f ComputeMedian();

  Eigen::Vector3f ComputeStdDev(const Eigen::Vector3f& median);

  int window_size_ = 30;
  int min_valid_frame_ = 10;
  std::deque<Eigen::Vector3f> history_;

  float threshold_ = 0.0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
