/**
 * @file ld_post_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线后处理工厂类
 * @version 0.1
 * @date 2024-10-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "ld_post_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

/**
 * @brief 车道线后处理工厂类
 * @details 通过类型创建对应的后处理类
 * @details 如果需要添加新的后处理类，只需要在Create函数中添加对应的创建逻辑
 */
class LdPostManager {
 public:
  /**
   * @brief 创建后处理类
   *
   * @param file 后处理配置文件路径
   * @return std::shared_ptr<LdPostBase> 后处理类指针
   */
  static std::shared_ptr<LdPostBase> Create(const std::string& file);

  /**
   * @brief 创建后处理类
   *
   * @param config 后处理配置
   * @return std::shared_ptr<LdPostBase> 后处理类指针
   */
  static std::shared_ptr<LdPostBase> Create(const YAML::Node& config);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END