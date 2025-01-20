#include "trunk_perception/app/target_fusion/data_fusion/type_fusion_sliding_window.h"
#include <memory>
#include <numeric>

#include "trunk_perception/common/error/code.hpp"
#include "trunk_perception/common/types/fused_object.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

TypeFusionSlidingWindow::TypeFusionSlidingWindow(const TypeFusionConfig::ConstPtr& config) : TypeFusionBase(config) {
  if (config->config_type != "SlidingWindow") {
    TERROR << "TypeFusionSlidingWindow::TypeFusionSlidingWindow: config type is not SlidingWindow";
    return;
  }

  TypeFusionSlidingWindowConfig::ConstPtr type_fusion_sliding_window_config =
      std::dynamic_pointer_cast<const TypeFusionSlidingWindowConfig>(config);
  if (!type_fusion_sliding_window_config) {
    TERROR
        << "TypeFusionSlidingWindowConfig::TypeFusionSlidingWindowConfig: type_fusion_sliding_window_config is nullptr";
    return;
  }

  window_size_ = type_fusion_sliding_window_config->window_size;
  min_valid_frame_ = type_fusion_sliding_window_config->min_valid_frame;

  if (window_size_ <= 0) {
    TERROR << "TypeFusionSlidingWindow::Init window_size must be greater than 0";
  }
}

// 这里初始化需要注意，目前仅对于激光测量新建航迹逻辑正确；若之后考虑视觉测量新建航迹，需要修改逻辑
std::uint32_t TypeFusionSlidingWindow::Init(const ObjectType& type) {
  object_type_ = type;
  lidar_type_history_.clear();
  lidar_type_history_.push_back(type);
  lidar_type_map_[type] = 1;
  return ErrorCode::SUCCESS;
}

std::uint32_t TypeFusionSlidingWindow::Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) {
  if (!sensor_measure_frame) {
    TERROR << "TypeFusionSlidingWindow::Update: sensor_measure_frame is nullptr";
    return ErrorCode::PARAMETER_ERROR;
  }

  if (sensor_measure_frame->sensor_type == MeasureSensorType::Lidar) {
    LidarMeasureFrame::ConstPtr lidar_measure_frame =
        std::dynamic_pointer_cast<const LidarMeasureFrame>(sensor_measure_frame);
    if (!lidar_measure_frame) {
      TERROR << "TypeFusionSlidingWindow::Update: Lidar measure_frame is nullptr";  // @zzg 2024-12-26
      return ErrorCode::PARAMETER_ERROR;
    }

    object_type_ = lidar_measure_frame->type;
    return SlidingWindow(lidar_type_history_, lidar_type_map_, object_type_);

  } else if (sensor_measure_frame->sensor_type == MeasureSensorType::FrontVision) {
    VisionMeasureFrame::ConstPtr front_vision_measure_frame =
        std::dynamic_pointer_cast<const VisionMeasureFrame>(sensor_measure_frame);

    if (!front_vision_measure_frame) {
      TERROR << "TypeFusionSlidingWindow::Update: FrontVision measure_frame is nullptr";  // @zzg 2024-12-26
      return ErrorCode::PARAMETER_ERROR;
    }
    object_type_ = front_vision_measure_frame->type;
    return SlidingWindow(front_vision_type_history_, front_vision_lidar_type_map_, object_type_);
  } else if (sensor_measure_frame->sensor_type == MeasureSensorType::SideVision) {
    SideVisionMeasureFrame::ConstPtr side_vision_measure_frame =
        std::dynamic_pointer_cast<const SideVisionMeasureFrame>(sensor_measure_frame);

    if (!side_vision_measure_frame) {
      TERROR << "TypeFusionSlidingWindow::Update: SideVision measure_frame is nullptr";  // @zzg 2024-12-26
      return ErrorCode::PARAMETER_ERROR;
    }
    object_type_ = side_vision_measure_frame->type;
    return SlidingWindow(side_vision_type_history_, side_vision_lidar_type_map_, object_type_);
  }
  return ErrorCode::SUCCESS;
}

std::uint32_t TypeFusionSlidingWindow::SlidingWindow(std::deque<ObjectType>& type_history,
                                                     std::unordered_map<ObjectType, int>& type_map,
                                                     ObjectType& measure_type) {
  type_history.push_back(measure_type);
  if (type_map.find(measure_type) != type_map.end()) {
    int num = type_map[measure_type];
    num += 1;
    type_map[measure_type] = num;
  } else {
    lidar_type_map_[measure_type] = 1;
  }

  if (type_history.size() < min_valid_frame_) {
    return ErrorCode::SUCCESS;
  }

  if (type_history.size() > window_size_) {
    ObjectType type = type_history.front();
    type_history.pop_front();
    if (type_map.find(type) != type_map.end()) {
      int num = type_map[type];
      num -= 1;
      type_map[measure_type] = num;
    }
  }

  int max_num = 0;
  ObjectType obj_type = measure_type;
  for (auto it = type_map.begin(); it != type_map.end(); ++it) {
    if (it->second > max_num) {
      max_num = it->second;
      obj_type = it->first;
    }
  }
  measure_type = obj_type;

  return ErrorCode::SUCCESS;
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END