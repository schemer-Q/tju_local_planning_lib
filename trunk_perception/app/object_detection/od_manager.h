/**
 * @file od_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 物体检测app工厂
 * @version 0.1
 * @date 2024-12-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include "od_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN


/**
 * @brief 物体检测app工厂
 */
class OdManager {
 public:
  /**
   * @brief 创建物体检测app
   * 
   * @param config 配置
   * @return std::shared_ptr<OdBase> 物体检测app指针
   */
  static std::shared_ptr<OdBase> Create(const YAML::Node& config);

  /**
   * @brief 创建物体检测app
   * 
   * @param file 配置文件路径
   * @return std::shared_ptr<OdBase> 物体检测app指针
   */
  static std::shared_ptr<OdBase> Create(const std::string& file);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END