/**
 * @file macros.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 宏定义
 * @version 0.1
 * @date 2024-09-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once


#define EPS 1e-6


#define TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN \
namespace trunk {                            \
namespace perception {

#define TRUNK_PERCEPTION_LIB_NAMESPACE_END \
}                                          \
}

#define TRUNK_PERCEPTION_LIB_NAMESPACE trunk::perception

#define TRUNK_PERCEPTION_LIB_APP_NAMESPACE_BEGIN \
namespace trunk {                            \
namespace perception {                     \
namespace app {

#define TRUNK_PERCEPTION_LIB_APP_NAMESPACE_END \
}                                          \
}                                          \
}

#define TRUNK_PERCEPTION_LIB_APP_NAMESPACE trunk::perception::app

#define TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_BEGIN \
namespace trunk {                            \
namespace perception {                     \
namespace common {

#define TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE_END \
}                                          \
}                                          \
}

#define TRUNK_PERCEPTION_LIB_COMMON_NAMESPACE trunk::perception::common

