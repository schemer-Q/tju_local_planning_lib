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

#include "common/macros.h"

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN

enum ErrorCode : uint32_t {
  SUCCESS = 0,
  DATA_BUFFER_ROLLBACK = 1,
  DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY = 2,
  DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT = 3,
};

const std::unordered_map<ErrorCode, std::string> error_code_to_string = {
    {ErrorCode::SUCCESS, "SUCCESS"},
    {ErrorCode::DATA_BUFFER_ROLLBACK, "DATA_BUFFER_ROLLBACK"},
    {ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY, "DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY"},
    {ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT, "DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT"},
};

[[__maybe_unused__]] static std::string get_error_code_string(const ErrorCode& error_code) {
  return error_code_to_string.at(error_code);
}

TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END