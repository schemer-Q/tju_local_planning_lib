#include "trunk_perception/app/lane_detection_post/algo/id_pool.h"
#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

IDPool::IDPool(const int& max_id) : max_id_(max_id) { Init(); }

void IDPool::Init() {
  valid_id_pool_.clear();
  for (int i = 0; i <= max_id_; i++) {
    valid_id_pool_.push_back(i);
  }
}

void IDPool::Init(const int& max_id) {
  max_id_ = max_id;
  Init();
}

int IDPool::GetNewId() {
  if (!valid_id_pool_.empty()) {
    int id = valid_id_pool_.front();
    valid_id_pool_.pop_front();
    return id;
  } else {
    TERROR << "IDPool::GetNewId() id pool running out!!!";
    return -1;
  }
}

void IDPool::RecycleId(const int& id) {
  if (0 <= id && id <= max_id_) {
    valid_id_pool_.push_back(id);
  } else {
    TERROR << "IDPool::RecycleId() id " << id << " invalid!!! max id:" << max_id_;
  }
}

void IDPool::Reset() { Init(); }
}

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
