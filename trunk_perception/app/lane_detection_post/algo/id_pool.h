/**
 * @file id_pool.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief ID池
 * @version 0.1
 * @date 2024-10-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <deque>
#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

class IDPool {
 public:
  IDPool() = default;
  explicit IDPool(const int& max_id);
  void Init(const int& max_id);
  int GetNewId();
  void RecycleId(const int& id);
  void Reset();

 private:
  void Init();

 private:
  std::deque<int> valid_id_pool_;
  int max_id_;
};

}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
