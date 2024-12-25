#include "trunk_perception/app/target_fusion/utils/id_pool.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

IDPool::IDPool(const unsigned int& max_id) : max_id_(max_id) {}

IDPool::~IDPool() = default;

// @author zzg 2024_12_25 修改 ID 分配方式
int IDPool::GetID() {
	int id;
	do {
		next_id_ += 1;
		next_id_ = next_id_ % max_id_;
	}while(IsIDAllocated(next_id_));

	allocated_ids_.insert(next_id_);
	id = next_id_;
	return id;
}

void IDPool::ReleaseID(const int& id) {
  if (allocated_ids_.find(id) == allocated_ids_.end()) {
    TERROR << "IDPool::ReleaseID ID " << id << " not allocated";
    return;
  }
  // released_ids_.push(id);
  allocated_ids_.erase(id);
}

bool IDPool::IsIDAllocated(const int& id) {
  return allocated_ids_.find(id) != allocated_ids_.end();
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END