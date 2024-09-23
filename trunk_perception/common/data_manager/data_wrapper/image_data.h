/**
 * @file image_data.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 图像数据封装
 * @version 0.1
 * @date 2024-09-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/common/tools/data_buffer.hpp"
#include "trunk_perception/common/types/image.h"

REGISTOR_SENSOR_DATA(ImageData, TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE::Image::ConstPtr)
