/**
 * @file id_pool.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief ID池
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stack>
#include <unordered_set>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

class IDPool {
 public:
  IDPool(const unsigned int& max_id);
  ~IDPool();

  int GetID();

  void ReleaseID(const int& id);

  bool IsIDAllocated(const int& id);

 private:
  unsigned int max_id_;  ///< 最大ID
  int next_id_;         ///< 下一个ID
  std::stack<int> released_ids_;  ///< 释放的ID池
  std::unordered_set<int> allocated_ids_;  ///< 已分配的ID集合
};

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END