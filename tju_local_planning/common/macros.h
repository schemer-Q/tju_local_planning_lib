#pragma once

#define EPSILON 1e-6

#define TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN \
  namespace tju {                          \
  namespace local_planning {

#define TJU_LOCAL_PLANNING_LIB_NAMESPACE_END \
  }                                         \
  }

#define TJU_LOCAL_PLANNING_LIB_NAMESPACE tju::local_planning

#define TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_BEGIN \
  namespace tju {                              \
  namespace local_planning {                          \
  namespace app {

#define TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE_END \
  }                                             \
  }                                             \
  }

#define TJU_LOCAL_PLANNING_LIB_APP_NAMESPACE tju::local_planning::app

#define TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN \
  namespace tju {                                 \
  namespace local_planning {                             \
  namespace common {

#define TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END \
  }                                                \
  }                                                \
  }

#define TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE tju::local_planning::common
