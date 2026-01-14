#pragma once

#include <vector>
#include <memory>

#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/common/types/header.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct OgmPoint {
 public:
  double x;     // og点x
  double y;     // og点y
  double z;     // og点z
  double p;     // og点p
  uint8_t type; // type

  typedef std::shared_ptr<OgmPoint> Ptr;
  typedef std::shared_ptr<const OgmPoint> ConstPtr;
};

struct OgmPoints {
 public:
  //uint64_t timestamp;  // 对应std_msgs/Header header
  Header header;
  std::vector<OgmPoint> points; // og point list

  typedef std::shared_ptr<OgmPoints> Ptr;
  typedef std::shared_ptr<const OgmPoints> ConstPtr;
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END