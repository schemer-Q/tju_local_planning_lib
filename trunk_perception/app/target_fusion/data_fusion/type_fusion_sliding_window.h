/**
 * @file type_fusion_sliding_window.h
 * @author zzg
 * @brief 基于滑动窗口滤波的类型融合 初版
 * @version 0.1
 * @date 2024-12-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <deque>
#include <unordered_map>

#include "type_fusion_base.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

struct TypeFusionSlidingWindowConfig : public TypeFusionConfig {
  int window_size;
  int min_valid_frame;

  typedef std::shared_ptr<TypeFusionSlidingWindowConfig> Ptr;
  typedef std::shared_ptr<const TypeFusionSlidingWindowConfig> ConstPtr;
};

/**
 * @brief 基于滑动窗口的类型融合实现
 */
class TypeFusionSlidingWindow : public TypeFusionBase {
 public:
  TypeFusionSlidingWindow(const TypeFusionConfig::ConstPtr& config);
  ~TypeFusionSlidingWindow() = default;

  std::uint32_t Init(const ObjectType& type) override;
  std::uint32_t Update(const SensorMeasureFrame::ConstPtr& sensor_measure_frame) override;
  std::uint32_t SlidingWindow(std::deque<ObjectType>& type_history, std::unordered_map<ObjectType, int>& type_map,
                              ObjectType& measure_type);

 private:
  int window_size_ = 20;
  int min_valid_frame_ = 10;
  std::deque<ObjectType> lidar_type_history_;
  std::unordered_map<ObjectType, int> lidar_type_map_;
  std::deque<ObjectType> front_vision_type_history_;
  std::unordered_map<ObjectType, int> front_vision_lidar_type_map_;
  std::deque<ObjectType> side_vision_type_history_;
  std::unordered_map<ObjectType, int> side_vision_lidar_type_map_;
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END