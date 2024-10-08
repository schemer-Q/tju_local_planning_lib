/**
 * @file ld_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线检测器管理类
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>

#include "lane_detector_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 车道线检测器管理类
 * @details 通过 Create 方法创建实例
 */
class LaneDetectorManager {
 public:
  /**
   * @brief 创建车道线检测器实例
   * @param file 配置文件路径
   * @return 车道线检测器实例
   */
  static std::shared_ptr<LaneDetectorBase> Create(const std::string& file);

  /**
   * @brief 创建车道线检测器实例
   * @param config 配置
   * @return 车道线检测器实例
   */
  static std::shared_ptr<LaneDetectorBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
