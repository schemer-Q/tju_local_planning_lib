/**
 * @file utils.hpp
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 系统工具函数
 * @version 0.1
 * @date 2024-09-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <string>
#include <filesystem>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

/**
 * @brief 判断文件是否存在
 * 
 * @param path [in] 文件路径
 * @return true 文件存在
 * @return false 文件不存在
 */
inline bool is_file_exist(const std::string& path) {
    return std::filesystem::exists(path);
}


TRUNK_PERCEPTION_LIB_NAMESPACE_END
