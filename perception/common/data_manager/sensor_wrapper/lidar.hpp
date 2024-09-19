/**
 * @file lidar.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 激光雷达传感器封装
 * @details 支持线程安全下的数据缓存、按照时间戳提取数据功能，支持不同类型的点云扩展,e.g. orig, tf, preprocessed, etc.
 * @details pose设置 线程不安全
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <sys/types.h>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/data_manager/data_wrapper/pointcloud_data.h"
#include "common/data_manager/sensor_wrapper/base.h"
#include "common/error/code.hpp"
#include "common/macros.h"
#include "common/types/point.h"
#include "tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

class Lidar : public SensorWrapper {
 public:
  struct MetaData {
    std::shared_ptr<Eigen::Isometry3f> pose_ptr;
  };

  Lidar() = default;
  ~Lidar() = default;

  /**
   * @brief 激光雷达支持有类型列表初始化
   */
  uint32_t init(const std::string& name, const std::vector<std::string>& types, const uint32_t& buffer_size,
                const double& max_time_delay) override {
    name_ = name;
    buffer_size_ = buffer_size;
    max_time_delay_ = max_time_delay;
    pointcloud_types_ = types;
    for (const auto& type : pointcloud_types_) {
      // 没有做重复类型检查，如果重复，会覆盖之前的类型
      m_type_buffer_[type] = std::make_shared<PointCloudDataBuffer>(type, buffer_size, max_time_delay);
    }
    meta_ = std::make_shared<MetaData>();
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 将数据推入缓冲区
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [in] 原始数据指针, e.g. std::shared_ptr<PointCloudT>. 注意pcl中的指针为boost::shared_ptr
   * @return uint32_t 错误码
   */
  virtual uint32_t push(const std::string& type, const double& timestamp,
                        const std::shared_ptr<const void>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      TERROR << "Lidar: " << name_ << " push failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    // 判断data类型为PointCloudT::Ptr
    const auto pointcloud_data = std::dynamic_pointer_cast<const PointCloudT>(data);
    if (!pointcloud_data) {
      TFATAL << "Lidar: " << name_ << " push failed for data type mismatch or nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }

    return it->second->push(PointCloudData(timestamp, pointcloud_data));
  }

  /**
   * @brief 按照时间戳提取数据
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [out] 传感器数据封装, e.g. std::shared_ptr<PointCloudData>
   * @return uint32_t 错误码
   */
  uint32_t extractByTime(const std::string& type, const double& timestamp, std::shared_ptr<void>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      TERROR << "Lidar: " << name_ << " extractByTime failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    auto pointcloud_data = std::make_shared<PointCloudData>();
    uint32_t result = it->second->extractByTime(timestamp, *pointcloud_data);
    data = pointcloud_data;
    return result;
  }

  /**
   * @brief 更新传感器元数据
   *
   * @param meta [in] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  uint32_t updateMeta(const std::shared_ptr<const void>& meta) override {
    const auto sensor_meta = std::dynamic_pointer_cast<const MetaData>(meta);
    if (!sensor_meta) {
      TFATAL << "Lidar: " << name_ << " updateMeta failed for data type mismatch or nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (!meta_) {
      TERROR << "Lidar: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta_->pose_ptr = sensor_meta->pose_ptr;
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 获取传感器元数据
   *
   * @param meta [out] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  uint32_t getMeta(std::shared_ptr<void>& meta) override {
    if (!meta_) {
      TERROR << "Lidar: " << name_ << " getMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta = std::make_shared<MetaData>(*meta_);
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 设置传感器位姿, 线程不安全
   *
   * @param pose [in] 位姿
   * @return 错误码
   */
  uint32_t setPose(const Eigen::Isometry3f& pose) {
    meta_->pose_ptr = std::make_shared<Eigen::Isometry3f>(pose);
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 获取传感器位姿, 线程不安全
   *
   * @return 位姿
   */
  std::shared_ptr<const Eigen::Isometry3f> pose() const { return meta_->pose_ptr; }

  /**
   * @brief 获取传感器名称
   *
   * @return 传感器名称
   */
  const std::string& name() const override { return name_; }

  /**
   * @brief 激光雷达不支持无类型列表初始化
   */
  uint32_t init(const std::string& name, const uint32_t& buffer_size, const double& max_time_delay) override {
    TFATAL << "Lidar: " << name << " init failed for no type list.";
    return ErrorCode::FUNCTION_NOT_IMPLEMENTED;
  }

  /**
   * @brief 激光雷达不支持无类型列表初始化
   */
  uint32_t push(const double& timestamp, const std::shared_ptr<const void>& data) override {
    TFATAL << "Lidar: " << name_ << " push failed for no type list.";
    return ErrorCode::FUNCTION_NOT_IMPLEMENTED;
  }

  /**
   * @brief 激光雷达不支持无类型列表初始化
   */
  uint32_t extractByTime(const double& timestamp, std::shared_ptr<void>& data) override {
    TFATAL << "Lidar: " << name_ << " extractByTime failed for no type list.";
    return ErrorCode::FUNCTION_NOT_IMPLEMENTED;
  }

 private:
  std::string name_;                           ///< 传感器名称
  uint32_t buffer_size_;                       ///< 数据缓冲区大小
  double max_time_delay_;                      ///< 数据缓冲区最大时间延迟
  std::vector<std::string> pointcloud_types_;  ///< 点云类型, 如: orig, tf, preprocessed, etc.

  std::unordered_map<std::string, PointCloudDataBufferPtr> m_type_buffer_;
  std::shared_ptr<MetaData> meta_;  ///< 传感器元数据
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
