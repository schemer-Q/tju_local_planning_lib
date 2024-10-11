#include "trunk_perception/app/lane_detection_post/algo/type_filter.h"
#include <vector>
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN

namespace ld_post {

TypeFilter::TypeFilter(const int& n_class, const size_t& max_cache_length)
    : n_class_(n_class), max_cache_length_(max_cache_length) {
  cls_indexes_ = std::vector<int>(n_class_, 0);
}

int TypeFilter::Update(const int& cls_idx) {
  if (cls_idx >= n_class_) {
    TERROR << "TypeFilter::Update: cls index overflow the max class number!!!";
    return cls_idx;
  }
  // push values
  cls_seq_indexes_.push(cls_idx);
  cls_indexes_[cls_idx]++;

  // check length
  if (cls_seq_indexes_.size() > max_cache_length_) {
    int idx = cls_seq_indexes_.front();
    cls_seq_indexes_.pop();
    cls_indexes_[idx]--;
  }

  // get max type value
  int max_rate_idx = 0;
  for (int i = 1; i < n_class_; i++) {
    if (cls_indexes_[i] > cls_indexes_[max_rate_idx]) {
      max_rate_idx = i;
    }
  }
  return max_rate_idx;
}

}  // namespace ld_post

TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END
