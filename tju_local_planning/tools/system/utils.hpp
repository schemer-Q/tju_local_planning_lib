#pragma once

#include <string>
#ifdef OLD_GCC
#include <experimental/filesystem>
#else
#include <filesystem>
#endif

#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

/**
 * @brief 判断文件是否存在
 * 
 * @param path [in] 文件路径
 * @return true 文件存在
 * @return false 文件不存在
 */
inline bool is_file_exist(const std::string& path) {
#ifdef OLD_GCC
    return std::experimental::filesystem::exists(path);
#else
    return std::filesystem::exists(path);
#endif
}


TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
