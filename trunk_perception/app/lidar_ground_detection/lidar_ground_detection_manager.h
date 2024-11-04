/**
 * @file lidar_ground_detection_manager.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief lidar ground detection manager
 * @version 0.1
 * @date 2024-11-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/app/lidar_ground_detection/lidar_ground_detection_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 激光雷达地面检测工厂类
 * @details 通过类型创建对应的地面检测类
 * @details 如果需要添加新的地面检测类，只需要在Create函数中添加对应的创建逻辑
 */
class LidarGroundDetectionManager {
 public:
  /**
   * @brief 创建地面检测类
   *
   * @param file 地面检测配置文件路径
   * @return std::shared_ptr<LidarGroundDetectionBase> 地面检测类指针
   */
  static std::shared_ptr<LidarGroundDetectionBase> Create(const std::string &file);

  /**
   * @brief 创建地面检测类
   *
   * @param config 地面检测配置
   * @return std::shared_ptr<LidarGroundDetectionBase> 地面检测类指针
   */
  static std::shared_ptr<LidarGroundDetectionBase> Create(const YAML::Node &config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
