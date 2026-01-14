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

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/tools/data_buffer.hpp"
#include "tju_local_planning/common/types/odometry.h"

REGISTOR_SENSOR_DATA(OdometryData, TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::Odometry::ConstPtr)
