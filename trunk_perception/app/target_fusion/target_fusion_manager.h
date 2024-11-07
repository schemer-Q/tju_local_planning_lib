  /**
 * @file target_fusion_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 目标后融合工厂类
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/app/target_fusion/target_fusion_base.h"


TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 目标后融合管理类
 *
 */
class TargetFusionManager {
 public:
  /**
   * @brief 创建目标后融合类
   *
   * @param config 配置
   * @return std::shared_ptr<TargetFusionBase> 目标后融合类指针
   */
  static std::shared_ptr<TargetFusionBase> Create(const YAML::Node& config);

  /**
   * @brief 创建目标后融合类
   *
   * @param config_file 配置文件路径
   * @return std::shared_ptr<TargetFusionBase>
   */
  static std::shared_ptr<TargetFusionBase> Create(const std::string& config_file);
};


TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END