#include "trunk_perception/app/target_fusion/utils/id_pool.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

IDPool::IDPool(const unsigned int& max_id) : max_id_(max_id) {}

IDPool::~IDPool() = default;

int IDPool::GetID() {
  int id;
  if (!released_ids_.empty()) {
    id = released_ids_.top();
    released_ids_.pop();
  } else {
    if (next_id_ >= max_id_) {
      TERROR << "IDPool::GetID MAX ID " << max_id_ << " reached";
      return -1;
    }
    id = next_id_++;
  }
  allocated_ids_.insert(id);
  return id;
}

void IDPool::ReleaseID(const int& id) {
  if (allocated_ids_.find(id) == allocated_ids_.end()) {
    TERROR << "IDPool::ReleaseID ID " << id << " not allocated";
    return;
  }
  released_ids_.push(id);
  allocated_ids_.erase(id);
}

bool IDPool::IsIDAllocated(const int& id) {
  return allocated_ids_.find(id) != allocated_ids_.end();
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END