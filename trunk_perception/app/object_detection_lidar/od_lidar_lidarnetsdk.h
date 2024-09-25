/**
 * @file od_lidar_lidarnetsdk.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 基于 lidarnetsdk 的激光雷达物体检测模型
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/object_detection_lidar/od_lidar_base.h"

namespace lidar_net {
class LidarNetDetector;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 基于 lidarnetsdk 的激光雷达物体检测模型
 *
 */
class OdLidarLidarNetSdk : public OdLidarBase {
 public:
  OdLidarLidarNetSdk();
  ~OdLidarLidarNetSdk();

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts = 0.0) override;
  std::any GetData(const std::string& key) override;

 private:
  std::shared_ptr<lidar_net::LidarNetDetector> detector_ = nullptr;
  std::string model_config_path_;
  std::string lidar_name_;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END