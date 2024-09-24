/**
 * @file app_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief app base class
 * @version 0.1
 * @date 2024-09-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <any>
#include <cstdint>
#include <yaml-cpp/yaml.h>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief app base class
 * @details 为所有感知应用的外部调用提供统一的接口
 * @details 应用内部通过DataManager获取数据
 */
class AppBase {
 public:
  AppBase() = default;
  virtual ~AppBase() = default;

  virtual std::uint32_t Init(const YAML::Node& config) = 0;
  virtual std::uint32_t Run(const double& ts = 0.0) = 0;
  virtual std::any GetData(const std::string& key) = 0;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
