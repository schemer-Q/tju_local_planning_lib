/**
 * @file side_od_vision_frame.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 环视视觉目标检测数据帧
 * @version 0.1
 * @date 2024-12-10
 *
 * @copyright Copyright (c) 2024
 *
 */


#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

struct SideOdVisionFrame {
  double timestamp = 0.0;                                ///< 时间戳, 单位为秒
  std::vector<Object> detected_objects;                  ///< 当前帧障碍物检测列表
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END