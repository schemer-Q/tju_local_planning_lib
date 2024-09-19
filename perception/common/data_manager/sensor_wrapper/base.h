/**
 * @file base.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 传感器包装类基类
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include "common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

/**
 * @brief 传感器包装类基类
 * @details 所有传感器包装类必须继承此类
 * @details 提供push和extract方法，用于将数据推入缓冲区，并按照时间戳提取数据
 * @details 提供meta数据get和set方法，用于获取和设置传感器元数据，如内外参等
 */
class SensorWrapper {
 public:
  SensorWrapper() = default;
  virtual ~SensorWrapper() = default;

  /**
   * @brief 初始化传感器, 使用类型列表
   * 
   * @param name [in] 传感器名称
   * @param types [in] 数据类型列表
   * @param buffer_size [in] 数据缓冲区大小
   * @param max_time_delay [in] 最大时间延迟, 单位为秒
   * @return uint32_t 错误码
   */
  virtual uint32_t init(const std::string& name, const std::vector<std::string>& types, const uint32_t& buffer_size,
                        const double& max_time_delay) = 0;

  /**
   * @brief 将数据推入缓冲区
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [in] 原始数据指针, e.g. PointCloudT::Ptr
   * @return uint32_t 错误码
   */
  virtual uint32_t push(const std::string& type, const double& timestamp, const std::shared_ptr<const void>& data) = 0;

  /**
   * @brief 按照时间戳提取数据
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [out] 传感器数据封装, e.g. std::shared_ptr<PointCloudData>
   * @return uint32_t 错误码
   */
  virtual uint32_t extractByTime(const std::string& type, const double& timestamp, std::shared_ptr<void>& data) = 0;

  /**
   * @brief 初始化传感器, 不使用类型列表
   * 
   * @param name [in] 传感器名称
   * @param buffer_size [in] 数据缓冲区大小
   * @param max_time_delay [in] 最大时间延迟, 单位为秒
   * @return uint32_t 错误码
   */
  virtual uint32_t init(const std::string& name, const uint32_t& buffer_size, const double& max_time_delay) = 0;
  
  /**
   * @brief 将数据推入缓冲区, 不使用类型
   * 
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [in] 原始数据指针, e.g. PointCloudT::Ptr
   * @return uint32_t 错误码
   */
  virtual uint32_t push(const double& timestamp, const std::shared_ptr<const void>& data) = 0;

  /**
   * @brief 按照时间戳提取数据, 不使用类型
   *
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [out] 传感器数据封装, e.g. std::shared_ptr<PointCloudData>
   * @return uint32_t 错误码
   */
  virtual uint32_t extractByTime(const double& timestamp, std::shared_ptr<void>& data) = 0;

  /**
   * @brief 更新传感器元数据
   * 
   * @param meta [in] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  virtual uint32_t updateMeta(const std::shared_ptr<const void>& meta) = 0;

  /**
   * @brief 获取传感器元数据
   * 
   * @param meta [out] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  virtual uint32_t getMeta(std::shared_ptr<void>& meta) = 0;

  /**
   * @brief 获取传感器名称
   * 
   * @return std::string 传感器名称
   */
  virtual const std::string& name() const = 0;

};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
