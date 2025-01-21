/**
 * @file od_front_mm_frame.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 前向多模态物体检测帧
 * @version 0.1
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/object.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

struct OdFrontMmFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp = 0.0;                ///< 时间戳, 单位为秒
  std::vector<Object> detected_objects;  ///< 模型检测障碍物列表
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END