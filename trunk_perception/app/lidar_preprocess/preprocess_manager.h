/**
 * @file preprocess_manager.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 激光雷达预处理工厂类
 * @version 0.1
 * @date 2024-09-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "preprocess_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN


/**
 * @brief 激光雷达预处理工厂类
 * @details 通过类型创建对应的预处理类
 * @details 如果需要添加新的预处理类，只需要在Create函数中添加对应的创建逻辑
 */
class PreprocessManager {
 public:
  /**
   * @brief 创建预处理类
   *
   * @param type 预处理类型, 如"BASE"
   * @return std::shared_ptr<PreprocessBase> 预处理类指针
   */
  static std::shared_ptr<PreprocessBase> Create(const std::string& type);
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
