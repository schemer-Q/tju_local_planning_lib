/**
 * @file radar.hpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 毫米波雷达传感器封装
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <cstdint>
#include <memory>

#include "gtest/gtest.h"
#include "trunk_perception/common/data_manager/data_wrapper/radar_data.h"
#include "trunk_perception/common/data_manager/sensor_wrapper/base.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

class ARS430RadarMetaInfo {
 public:
  std::shared_ptr<Eigen::Isometry3f> pose_ptr;  ///< 雷达外参
};

class ARS430Radar : public SensorWrapper<ars430::RadarObjects, ARS430RadarData, ARS430RadarMetaInfo> {
 public:
  ARS430Radar() = default;
  ~ARS430Radar() = default;

  uint32_t init(const std::string& name, const std::vector<std::string>& types, const uint32_t& buffer_size,
                const double& max_time_delay) override {
    (void)types;

    name_ = name;
    buffer_size_ = buffer_size;
    max_time_delay_ = max_time_delay;
    buffer_ = std::make_shared<ARS430RadarDataBuffer>(buffer_size, name_, max_time_delay);
    meta_ = std::make_shared<ARS430RadarMetaInfo>();
    return ErrorCode::SUCCESS;
  }

  uint32_t push(const std::string& type, const double& timestamp,
                const std::shared_ptr<ars430::RadarObjects>& data) override {
    if (!data) {
      TFATAL << "ARS430Radar: " << name_ << " push failed for data is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (!buffer_) {
      TERROR << "ARS430Radar: " << name_ << " push failed for buffer is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    (void)type;
    return buffer_->push(ARS430RadarData(timestamp, data));
  }

  uint32_t extractByTime(const std::string& type, const double& timestamp,
                         std::shared_ptr<ARS430RadarData>& data) override {
    if (!buffer_) {
      TERROR << "ARS430Radar: " << name_ << " extractByTime failed for buffer is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    (void)type;
    data = std::make_shared<ARS430RadarData>();
    uint32_t result = buffer_->extractByTime(timestamp, *data);
    if (result != ErrorCode::SUCCESS) {
      TERROR << "ARS430Radar: " << name_ << " extractByTime failed for extract failed.";
      data = nullptr;
    }
    return result;
  }

  uint32_t updateMeta(const std::shared_ptr<ARS430RadarMetaInfo>& meta) override {
    if (!meta) {
      TERROR << "ARS430Radar: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }
    if (!meta_) {
      TERROR << "ARS430Radar: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta_ = meta;
    return ErrorCode::SUCCESS;
  }

  uint32_t updateMeta(const std::string& name, const std::shared_ptr<void>& info) override {
    if (!meta_) {
      TERROR << "ARS430Radar: " << name_ << " updateMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    if (!info) {
      TERROR << "ARS430Radar: " << name_ << " updateMeta failed for info is nullptr.";
      return ErrorCode::PARAMETER_ERROR;
    }

    if (name == "pose") {
      const auto pose_ptr = std::static_pointer_cast<Eigen::Isometry3f>(info);
      if (!pose_ptr) {
        TERROR << "ARS430Radar: " << name_ << " updateMeta failed for pose_ptr is nullptr.";
        return ErrorCode::PARAMETER_ERROR;
      }
      meta_->pose_ptr = pose_ptr;
    } else {
      TERROR << "ARS430Radar: " << name_ << " updateMeta failed for name not found.";
      return ErrorCode::PARAMETER_ERROR;
    }
    return ErrorCode::SUCCESS;
  }

  uint32_t getMeta(std::shared_ptr<ARS430RadarMetaInfo>& meta) override {
    if (!meta_) {
      TERROR << "ARS430Radar: " << name_ << " getMeta failed for meta is nullptr.";
      return ErrorCode::UNINITIALIZED;
    }
    meta = meta_;
    return ErrorCode::SUCCESS;
  }

  uint32_t setPose(const Eigen::Isometry3f& pose) override {
    meta_->pose_ptr = std::make_shared<Eigen::Isometry3f>(pose);
    return ErrorCode::SUCCESS;
  }

  std::shared_ptr<const Eigen::Isometry3f> pose() const override { return meta_->pose_ptr; }

  const std::string& name() const override { return name_; }

  double getLatestOriginDataTime() { return buffer_->getLatestDataTime(); }

 private:
  std::string name_ = "FRONT_RADAR";
  uint32_t buffer_size_;
  double max_time_delay_;
  std::shared_ptr<ARS430RadarMetaInfo> meta_;
  ARS430RadarDataBufferPtr buffer_;
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END