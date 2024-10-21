/**
 * @file radar_data.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 毫米波雷达数据封装
 * @version 0.1
 * @date 2024-10-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/tools/data_buffer.hpp"
#include "trunk_perception/common/types/radar_ars430.h"
#include "trunk_perception/common/types/radar_cr5tp.h"

REGISTOR_SENSOR_DATA(ARS430RadarData, TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::ars430::RadarObjects::ConstPtr)
REGISTOR_SENSOR_DATA(CR5TPRadarData, TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::cr5tp::RadarObjects::ConstPtr)