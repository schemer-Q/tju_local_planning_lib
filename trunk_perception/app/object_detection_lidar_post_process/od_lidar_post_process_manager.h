/**
 * @file od_lidar_post_process_manager.h
 * @author Fan Dongsheng
 * @brief 激光检测后处理工厂创建管理器
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "od_lidar_post_process_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 激光检测后处理工厂
 *
 */
class OdLidarPostProcessManager {
 public:
  /**
   * @brief 创建物体检测后处理器
   *
   * @param file 配置文件路径
   * @return std::shared_ptr<OdLidarPostProcessBase> 物体检测后处理器指针
   */
  static std::shared_ptr<OdLidarPostProcessBase> Create(const std::string& file);

  /**
   * @brief 创建物体检测后处理器
   *
   * @param config 配置
   * @return std::shared_ptr<OdLidarPostProcessBase> 物体检测后处理器指针
   */
  static std::shared_ptr<OdLidarPostProcessBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END