/**
 * @file od_lidar_post_process_base.h
 * @author Fan Dongsheng
 * @brief 激光检测后处理基类
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 激光后处理基类
 *
 */
class OdLidarPostProcessBase : public AppBase {
 public:
  OdLidarPostProcessBase() = default;
  ~OdLidarPostProcessBase() override = default;

  /**
   * @brief object detection lidar post process init param
   * 
   * @param config yaml node
   * @return std::uint32_t 
   */
  std::uint32_t Init(const YAML::Node& config) override = 0;

  /**
   * @brief object detection lidar post process run pipeline
   * 
   * @param ts current timestamp
   * @return std::uint32_t 
   */
  std::uint32_t Run(const double& ts = 0.0) override = 0;

  /**
   * @brief Get the Data object
   * 
   * @param key string key
   * @return std::any 
   */
  std::any GetData(const std::string& key) override = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END