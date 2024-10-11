/**
 * @file ld_frame.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 车道线检测帧
 * @version 0.1
 * @date 2024-09-29
 *
 * @copyright Copyright (c) 2024
 *
 */


#pragma once

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/lane.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

struct LDFrame {
    double timestamp = 0.0; ///< 时间戳, 单位为秒
    std::vector<LaneLineVision> lanes_detected;  ///< 当前帧检测结果
    std::vector<LaneLineVision> lanes_tracked;  ///< 当前帧跟踪结果
};

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END