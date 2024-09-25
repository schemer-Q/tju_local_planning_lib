/**
 * @file od_lidar_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "od_lidar_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 基于激光模型的物体检测器工厂
 * 
 */
class OdLidarManager {
 public:
  /**
   * @brief 创建物体检测器
   * 
   * @param file 配置文件路径
   * @return std::shared_ptr<OdLidarBase> 物体检测器指针
   */
  static std::shared_ptr<OdLidarBase> Create(const std::string& file);

  /**
   * @brief 创建物体检测器
   * 
   * @param config 配置
   * @return std::shared_ptr<OdLidarBase> 物体检测器指针
   */
  static std::shared_ptr<OdLidarBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
