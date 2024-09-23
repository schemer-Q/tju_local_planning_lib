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

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/point.h"
#include "trunk_perception/common/tools/data_buffer.hpp"


REGISTOR_SENSOR_DATA(PointCloudData, PointCloudConstPtr)
