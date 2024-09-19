/**
 * @file pointcloud_data.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 点云数据封装
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "common/macros.h"
#include "common/types/point.h"
#include "common/tools/data_buffer.hpp"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

REGISTOR_SENSOR_DATA(PointCloudData, const PointCloudConstPtr)

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END