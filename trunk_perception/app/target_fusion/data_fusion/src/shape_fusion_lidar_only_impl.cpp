#include "trunk_perception/app/target_fusion/data_fusion/shape_fusion_lidar_only_impl.h"
#include <memory>
#include <numeric>

#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

ShapeFusionLidarOnlyImpl::ShapeFusionLidarOnlyImpl(const ShapeFusionConfig::ConstPtr& config)
    : ShapeFusionBase(config) {
  if (config->type != "LidarOnly") {
    TFATAL << "ShapeFusionLidarOnlyImpl::ShapeFusionLidarOnlyImpl: config type is not LidarOnly";
    return;
  }

  ShapeFusionLidarOnlyConfig::ConstPtr shape_fusion_lidar_only_config =
      std::dynamic_pointer_cast<const ShapeFusionLidarOnlyConfig>(config);
  if (!shape_fusion_lidar_only_config) {
    TFATAL << "ShapeFusionLidarOnlyImpl::ShapeFusionLidarOnlyImpl: shape_fusion_lidar_only_config is nullptr";
    return;
  }

  window_size_ = shape_fusion_lidar_only_config->window_size;
  threshold_ = shape_fusion_lidar_only_config->threshold;
  min_valid_frame_ = shape_fusion_lidar_only_config->min_valid_frame;

  if (window_size_ <= 0) {
    TFATAL << "ShapeFusionLidarOnlyImpl::Init window_size must be greater than 0";
  }
}

std::uint32_t ShapeFusionLidarOnlyImpl::Init(const Eigen::Vector3f& size) {
  size_ = size;
  history_.clear();
  history_.push_back(size);
  return ErrorCode::SUCCESS;
}

std::uint32_t ShapeFusionLidarOnlyImpl::Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) {
  if (sensor_measure_frame->sensor_type != MeasureSensorType::Lidar) {
    TFATAL << "ShapeFusionLidarOnlyImpl::Update: sensor_measure_frame is not lidar";
    return ErrorCode::PARAMETER_ERROR;
  }

  if (!sensor_measure_frame) {
    TERROR << "ShapeFusionLidarOnlyImpl::Update: sensor_measure_frame is nullptr";
    return ErrorCode::PARAMETER_ERROR;
  }

  LidarMeasureFrame::ConstPtr lidar_measure_frame =
      std::dynamic_pointer_cast<const LidarMeasureFrame>(sensor_measure_frame);

  if (!lidar_measure_frame) {
    TFATAL << "ShapeFusionLidarOnlyImpl::Update: lidar_measure_frame is nullptr";
    return ErrorCode::PARAMETER_ERROR;
  }

  size_ = lidar_measure_frame->size;
  history_.push_back(size_);
  if (history_.size() > window_size_) {
    history_.pop_front();
  }

  // 低于该阈值，则不使用历史帧进行滤波
  if (history_.size() < min_valid_frame_) {
    return ErrorCode::SUCCESS;
  }

  // 计算中位数和标准差
  Eigen::Vector3f median = ComputeMedian();
  Eigen::Vector3f std_dev = ComputeStdDev(median);

  bool is_normal = true;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(size_[i] - median[i]) > threshold_ * std_dev[i]) {
      is_normal = false;
      break;
    }
  }

  size_ = is_normal ? size_ : median;

  return ErrorCode::SUCCESS;
}

Eigen::Vector3f ShapeFusionLidarOnlyImpl::ComputeMedian() {
  Eigen::Vector3f median;
  for (int i = 0; i < 3; ++i) {
    std::vector<float> values;
    for (const auto& vec : history_) {
      values.push_back(vec[i]);
    }
    std::sort(values.begin(), values.end());
    int mid = values.size() / 2;
    median[i] = (values.size() % 2 == 0) ? (values[mid - 1] + values[mid]) / 2 : values[mid];
  }
  return median;
}

Eigen::Vector3f ShapeFusionLidarOnlyImpl::ComputeStdDev(const Eigen::Vector3f& median) {
  Eigen::Vector3f std_dev;
  for (int i = 0; i < 3; ++i) {
    std_dev[i] = std::sqrt(std::accumulate(history_.begin(), history_.end(), 0.0,
                                           [i, median](double sum, const Eigen::Vector3f& vec) {
                                             return sum + std::pow(vec[i] - median[i], 2);
                                           }) /
                           history_.size());
  }
  return std_dev;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
