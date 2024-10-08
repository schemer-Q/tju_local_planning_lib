/**
 * @file odometry_data.h
 * @author Fan Dongsheng
 * @brief 
 * @version 0.1
 * @date 2024-09-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <memory>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/types/odometry.h"
#include "trunk_perception/common/tools/data_buffer.hpp"


REGISTOR_SENSOR_DATA(OdometryData, TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::Odometry::ConstPtr)