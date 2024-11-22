/**
 * @file lidar_ground_detection_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief lidar ground detection base
 * @version 0.1
 * @date 2024-11-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/lidar/ground_detection/ground_detection_manager.h"
#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 激光地面分割处理基类
 *
 */
class LidarGroundDetectionBase : public AppBase {
 public:
  LidarGroundDetectionBase() = default;
  ~LidarGroundDetectionBase() = default;

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;

 private:
  std::string lidar_name_ = "";
  PointCloudPtr ground_cloud_ = nullptr;
  PointCloudPtr no_ground_cloud_ = nullptr;
  bool ground_detection_switch_ = false;
  std::shared_ptr<GroundDetectionBase> ground_detector_ = nullptr;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
