#pragma once

#include <memory>
#include <vector>

#include "tju_local_planning/common/macros.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

struct Header {
    public:
    uint32_t seq;
    uint64_t stamp;
    std::string frame_id;

    typedef std::shared_ptr<Header> Ptr;
    typedef std::shared_ptr<const Header> ConstPtr;

};



TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END