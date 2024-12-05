/**
 * @file fod_vision_post_process_manager.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-12-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/app/object_detection_front_vision_post_process/fod_vision_post_process_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class FodVisionPostProcessManager {
 public:
  /**
   * @brief 创建前向视觉检测后处理器
   *
   * @param file 配置文件路径
   * @return std::shared_ptr<FodVisionPostProcessBase> 前向视觉检测后处理器指针
   */
  static std::shared_ptr<FodVisionPostProcessBase> Create(const std::string& file);

  /**
   * @brief 创建前向视觉检测后处理器
   *
   * @param config 配置
   * @return std::shared_ptr<FodVisionPostProcessBase> 前向视觉检测后处理器指针
   */
  static std::shared_ptr<FodVisionPostProcessBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END