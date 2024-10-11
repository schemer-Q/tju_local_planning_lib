/**
 * @file type_filter.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 类别平滑窗口
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <queue>
#include <vector>
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

/**
 * @brief 基于平滑窗口的类别过滤器
 * 
 */
class TypeFilter {
 public:
  /**
   * @brief 构造函数
   *
   * @param n_class 类别数
   * @param max_cache_length 平滑窗口长度
   */
  TypeFilter(const int& n_class, const size_t& max_cache_length = 40);
  ~TypeFilter() {}

  /**
   * @brief 更新类别
   *
   * @param cls_idx 类别索引
   * @return int 更新后的类别索引
   */
  int Update(const int& cls_idx);

 private:
  int n_class_;
  size_t max_cache_length_;
  std::vector<int> cls_indexes_;
  std::queue<int> cls_seq_indexes_;
};
}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
