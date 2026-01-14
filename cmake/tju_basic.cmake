# jian
find_package(PkgConfig REQUIRED)
pkg_search_module(JIAN REQUIRED IMPORTED_TARGET jian)
include_directories(${JIAN_INCLUDE_DIRS})