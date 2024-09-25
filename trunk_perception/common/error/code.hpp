/**
 * @file code.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 错误码
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <string>
#include <unordered_map>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

enum ErrorCode : uint32_t {
  SUCCESS = 0,
  // 基础错误 [1-9]
  FUNCTION_NOT_IMPLEMENTED = 1,  ///< 函数未实现
  PARAMETER_ERROR = 2,           ///< 参数错误
  UNINITIALIZED = 3,             ///< 未正确初始化
  YAML_CONFIG_ERROR = 4,         ///< YAML配置文件错误
  // 数据缓冲区错误 [10-19]
  DATA_BUFFER_ROLLBACK = 10,
  DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY = 11,
  DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT = 12,
  // 传感器错误 [20-29]
  SENSOR_DATA_TYPE_NOT_FOUND = 20,
  SENSOR_POSE_NOT_FOUND = 21,
  POINT_CLOUD_INVALID = 22,
  // Object Detection Lidar 错误 [30-39]
  LIDAR_NET_SDK_INIT_FAILED = 30,
};

const std::unordered_map<ErrorCode, std::string> error_code_to_string = {
    {ErrorCode::SUCCESS, "SUCCESS"},
    {ErrorCode::FUNCTION_NOT_IMPLEMENTED, "FUNCTION_NOT_IMPLEMENTED"},
    {ErrorCode::PARAMETER_ERROR, "PARAMETER_ERROR"},
    {ErrorCode::UNINITIALIZED, "UNINITIALIZED"},
    {ErrorCode::YAML_CONFIG_ERROR, "YAML_CONFIG_ERROR"},
    {ErrorCode::DATA_BUFFER_ROLLBACK, "DATA_BUFFER_ROLLBACK"},
    {ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY, "DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY"},
    {ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT, "DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT"},
    {ErrorCode::SENSOR_DATA_TYPE_NOT_FOUND, "SENSOR_DATA_TYPE_NOT_FOUND"},
    {ErrorCode::SENSOR_POSE_NOT_FOUND, "SENSOR_POSE_NOT_FOUND"},
    {ErrorCode::POINT_CLOUD_INVALID, "POINT_CLOUD_INVALID"},
    {ErrorCode::LIDAR_NET_SDK_INIT_FAILED, "LIDAR_NET_SDK_INIT_FAILED"},
};

[[__maybe_unused__]] static std::string get_error_code_string(const ErrorCode& error_code) {
  return error_code_to_string.at(error_code);
}

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END