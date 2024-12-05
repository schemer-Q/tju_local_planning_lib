/**
 * @file fod_vision_frame.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief 前向视觉目标检测数据帧
 * @version 0.1
 * @date 2024-11-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

struct FodVisionFrame {
  double timestamp = 0.0;                                ///< 时间戳, 单位为秒
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();  ///< 前后帧的变换位姿
  std::vector<Object> detected_objects;                  ///< 当前帧障碍物检测列表
  std::vector<Object> tracked_objects;                   ///< 跟踪后的障碍物列表
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END