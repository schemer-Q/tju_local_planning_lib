#pragma once

#include <sys/types.h>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "tju_local_planning/common/data_manager/data_wrapper/pointcloud_data.h"
#include "tju_local_planning/common/data_manager/sensor_wrapper/base.h"
#include "tju_local_planning/common/error/code.hpp"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/point.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

class LidarMetaInfo {
 public:
  std::shared_ptr<Eigen::Isometry3f> pose_ptr;
};

class Lidar : public SensorWrapper<PointCloudT, PointCloudData, LidarMetaInfo> {
 public:
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
      m_type_buffer_[type] = std::make_shared<PointCloudDataBuffer>(buffer_size, name_, max_time_delay);
    }
    meta_ = std::make_shared<LidarMetaInfo>();
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 将数据推入缓冲区
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [in] 原始数据指针
   * @return uint32_t 错误码
   */
  uint32_t push(const std::string& type, const double& timestamp, const std::shared_ptr<PointCloudT>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      NTERROR << "Lidar: " << name_ << " push failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    if (!data) {
      NTFATAL << "Lidar: " << name_ << " push failed for data is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }

    return it->second->push(PointCloudData(timestamp, data));
  }

  /**
   * @brief 按照时间戳提取数据
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [out] 传感器数据
   * @return uint32_t 错误码
   */
  uint32_t extractByTime(const std::string& type, const double& timestamp,
                         std::shared_ptr<PointCloudData>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      NTERROR << "Lidar: " << name_ << " extractByTime failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    data = std::make_shared<PointCloudData>();
    uint32_t result = it->second->extractByTime(timestamp, *data);
    if (result != ErrorCode::SUCCESS) {
      NTERROR << "Lidar: " << name_ << " extractByTime failed for extract failed.";
      data = nullptr;
    }
    return result;
  }

  /**
   * @brief 更新传感器元数据
   *
   * @param meta [in] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  uint32_t updateMeta(const std::shared_ptr<LidarMetaInfo>& meta) override {
    if (!meta) {
      NTFATAL << "Lidar: " << name_ << " updateMeta failed for input meta is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (!meta_) {
      NTERROR << "Lidar: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta_ = meta;
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 更新传感器元数据
   *
   * @param name [in] info名称
   * @param info [in] info数据指针, e.g. std::shared_ptr<Eigen::Isometry3f>
   * @return uint32_t 错误码
   */
  uint32_t updateMeta(const std::string& name, const std::shared_ptr<void>& info) override {
    if (name == "pose") {
      meta_->pose_ptr = std::static_pointer_cast<Eigen::Isometry3f>(info);
    } else {
      NTERROR << "Lidar: " << name_ << " updateMeta failed for name not found.";
      return ErrorCode::PARAMETER_ERROR;
    }
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 获取传感器元数据
   *
   * @param meta [out] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  uint32_t getMeta(std::shared_ptr<LidarMetaInfo>& meta) override {
    if (!meta_) {
      NTERROR << "Lidar: " << name_ << " getMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta = std::make_shared<LidarMetaInfo>(*meta_);
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 设置传感器位姿, 线程不安全
   *
   * @param pose [in] 位姿
   * @return 错误码
   */
  uint32_t setPose(const Eigen::Isometry3f& pose) override {
    meta_->pose_ptr = std::make_shared<Eigen::Isometry3f>(pose);
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 获取传感器位姿, 线程不安全
   *
   * @return 位姿
   */
  std::shared_ptr<const Eigen::Isometry3f> pose() const override { return meta_->pose_ptr; }

  /**
   * @brief 获取传感器名称
   *
   * @return 传感器名称
   */
  const std::string& name() const override { return name_; }

  /**
   * @brief 获取传感器原始数据最新时间
   *
   * @return double 时间
   */
  double getLatestOriginDataTime() {
    const std::string type = "origin";
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型，返回异常值-1.0
      NTERROR << "Lidar: " << name_ << " getLatestOriginDataTime failed for type not found.";
      return -1.0;
    }

    return m_type_buffer_[type]->getLatestDataTime();
  }

 private:
  std::string name_;                           ///< 传感器名称
  uint32_t buffer_size_;                       ///< 数据缓冲区大小
  double max_time_delay_;                      ///< 数据缓冲区最大时间延迟
  std::vector<std::string> pointcloud_types_;  ///< 点云类型, 如: orig, tf, preprocessed, etc.

  std::unordered_map<std::string, PointCloudDataBufferPtr> m_type_buffer_;
  std::shared_ptr<LidarMetaInfo> meta_;  ///< 传感器元数据
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END
