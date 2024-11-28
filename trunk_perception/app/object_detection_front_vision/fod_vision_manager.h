/**
 * @file fod_vision_manager.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 前向视觉物体检测器工厂
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/app/object_detection_front_vision/fod_vision_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 前向视觉物体检测器工厂
 *
 */
class FodVisionManager {
 public:
  /**
   * @brief 创建物体检测器
   *
   * @param file 配置文件路径
   * @return std::shared_ptr<FodVisionBase> 物体检测器指针
   */
  static std::shared_ptr<FodVisionBase> Create(const std::string& file);

  /**
   * @brief 创建物体检测器
   *
   * @param config 配置
   * @return std::shared_ptr<FodVisionBase> 物体检测器指针
   */
  static std::shared_ptr<FodVisionBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
