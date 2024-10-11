/**
 * @file ld_post_base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线后处理基类
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/app/base/app_base.h"
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 车道线后处理基类
 * @details 用户可以继承该类，提供不同的后处理逻辑
 */
class LdPostBase : public AppBase {
 public:
  LdPostBase();
  ~LdPostBase();

  std::uint32_t Init(const YAML::Node& config) override;
  std::uint32_t Run(const double& ts) override;
  std::any GetData(const std::string& key) override;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END