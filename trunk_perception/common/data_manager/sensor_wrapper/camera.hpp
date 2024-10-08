/**
 * @file camera.hpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 相机传感器封装
 * @version 0.1
 * @date 2024-09-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "trunk_perception/common/data_manager/data_wrapper/image_data.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/tools/camera_undistort.hpp"
#include "trunk_perception/common/tools/standard_camera_projection.hpp"
#include "trunk_perception/common/types/camera_info.h"
#include "trunk_perception/common/types/image.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

class CameraMetaInfo {
 public:
  std::shared_ptr<Eigen::Isometry3f> pose_ptr;  ///< 相机外参
  CameraInfo::Ptr camera_info_ptr;              ///< 相机内外参, 包括畸变系数
};

class Camera : public SensorWrapper<Image, ImageData, CameraMetaInfo> {
 public:
  Camera() = default;
  ~Camera() = default;

  uint32_t init(const std::string& name, const std::vector<std::string>& types, const uint32_t& buffer_size,
                const double& max_time_delay) override {
    name_ = name;
    buffer_size_ = buffer_size;
    max_time_delay_ = max_time_delay;
    types_ = types;
    for (const auto& type : types_) {
      m_type_buffer_[type] = std::make_shared<ImageDataBuffer>(buffer_size, name_, max_time_delay);
    }
    meta_ = std::make_shared<CameraMetaInfo>();
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
  uint32_t push(const std::string& type, const double& timestamp, const std::shared_ptr<Image>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      TERROR << "Camera: " << name_ << " push failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    if (!data) {
      TFATAL << "Camera: " << name_ << " push failed for data is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    return it->second->push(ImageData(timestamp, data));
  }

  /**
   * @brief 按照时间戳提取数据
   *
   * @param type [in] 数据类型
   * @param timestamp [in] 时间戳, 单位为秒
   * @param data [out] 传感器数据封装, e.g. std::shared_ptr<PointCloudData>
   * @return uint32_t 错误码
   */
  uint32_t extractByTime(const std::string& type, const double& timestamp, std::shared_ptr<ImageData>& data) override {
    auto it = m_type_buffer_.find(type);
    if (it == m_type_buffer_.end()) {  // 没有找到对应的类型
      TERROR << "Camera: " << name_ << " extractByTime failed for type not found.";
      return ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND;
    }
    data = std::make_shared<ImageData>();
    uint32_t result = it->second->extractByTime(timestamp, *data);
    if (result != ErrorCode::SUCCESS) {
      TERROR << "Camera: " << name_ << " extractByTime failed for extract failed.";
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
  uint32_t updateMeta(const std::shared_ptr<CameraMetaInfo>& meta) override {
    if (!meta) {
      TFATAL << "Camera: " << name_ << " updateMeta failed for input meta is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (!meta_) {
      TERROR << "Camera: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta_ = meta;
    initProjection();
    initUndistort();
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
    if (!meta_) {
      TERROR << "Camera: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    if (!info) {
      TERROR << "Camera: " << name_ << " updateMeta failed for info is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }

    if (name == "pose") {
      const auto pose_ptr = std::static_pointer_cast<Eigen::Isometry3f>(info);
      if (!pose_ptr) {
        TERROR << "Camera: " << name_ << " updateMeta failed for pose_ptr is nullptr.";
        return ErrorCode::PARAMETER_ERROR;
      }
      meta_->pose_ptr = pose_ptr;
    } else if (name == "camera_info") {
      const auto camera_info_ptr = std::static_pointer_cast<CameraInfo>(info);
      if (!camera_info_ptr) {
        TERROR << "Camera: " << name_ << " updateMeta failed for camera_info_ptr is nullptr.";
        return ErrorCode::PARAMETER_ERROR;
      }
      meta_->camera_info_ptr = camera_info_ptr;
    } else {
      TERROR << "Camera: " << name_ << " updateMeta failed for name not found.";
      return ErrorCode::PARAMETER_ERROR;
    }
    initProjection();
    initUndistort();
    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 获取传感器元数据
   *
   * @param meta [out] 传感器元数据指针, e.g. std::shared_ptr<SensorMeta>
   * @return uint32_t 错误码
   */
  uint32_t getMeta(std::shared_ptr<CameraMetaInfo>& meta) override {
    if (!meta_) {
      TERROR << "Camera: " << name_ << " getMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta = std::make_shared<CameraMetaInfo>(*meta_);
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
    initProjection();
    return ErrorCode::SUCCESS;
  }

  void initProjection() {
    if (camera_projection_) return;
    if (meta_ && meta_->camera_info_ptr && meta_->pose_ptr) {
      camera_projection_ = std::make_shared<StandardCameraProjection>(*meta_->camera_info_ptr, *meta_->pose_ptr);
    }
  }

  void initUndistort() {
    if (camera_undistort_) return;
    if (meta_ && meta_->camera_info_ptr) {
      camera_undistort_ = std::make_shared<CameraUndistort>(*meta_->camera_info_ptr);
    }
  }

  std::shared_ptr<CameraUndistort> getUndistort() { return camera_undistort_; }

  std::shared_ptr<StandardCameraProjection> getProjection() { return camera_projection_; }

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

 private:
  std::string name_;                                                   ///< 传感器名称
  uint32_t buffer_size_;                                               ///< 数据缓冲区大小
  double max_time_delay_;                                              ///< 数据缓冲区最大时间延迟
  std::vector<std::string> types_;                                     ///< 图像类型, 如: orig, undistorted, etc.
  std::unordered_map<std::string, ImageDataBufferPtr> m_type_buffer_;  ///< 图像数据缓冲区
  std::shared_ptr<CameraMetaInfo> meta_;                               ///< 传感器元数据
  std::shared_ptr<StandardCameraProjection> camera_projection_ = nullptr;  ///< 相机投影工具
  std::shared_ptr<CameraUndistort> camera_undistort_ = nullptr;                     ///< 相机去畸变工具
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END
