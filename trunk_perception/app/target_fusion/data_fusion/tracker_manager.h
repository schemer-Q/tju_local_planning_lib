/**
 * @file tracker_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 负责创建tracker
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>

#include "trunk_perception/app/target_fusion/data_fusion/tracker.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/fused_object.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class TrackerManager {
 public:
  TrackerManager() = default;
  ~TrackerManager() = default;

  std::uint32_t Init(const YAML::Node& config);

  /**
   * @brief 基于激光雷达观测创建tracker
   *
   * @param track_id
   * @param lidar_object
   * @return TrackerPtr
   */
  TrackerPtr CreateTracker(const int& track_id, const LidarMeasureFrame::ConstPtr& lidar_object);

 private:
  MotionFusionConfig motion_kf_config_;
  ShapeFusionConfig::Ptr shape_fusion_config_ = nullptr;
  ExistenceFusionConfig::Ptr existence_fusion_config_ = nullptr;
  TypeFusionConfig::Ptr type_fusion_config_ = nullptr;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END