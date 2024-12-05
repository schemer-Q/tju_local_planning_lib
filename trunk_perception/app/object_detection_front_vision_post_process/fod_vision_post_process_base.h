/**
 * @file fod_vision_post_process_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 
 * @version 0.1
 * @date 2024-12-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class FodVisionPostProcessBase : public AppBase {
 public:
  FodVisionPostProcessBase() = default;
  ~FodVisionPostProcessBase() override = default;

  /**
   * @brief object detection front vision post process init param
   *
   * @param config yaml node
   * @return std::uint32_t
   */
  std::uint32_t Init(const YAML::Node& config) override = 0;

  /**
   * @brief object detection front vision post process run pipeline
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