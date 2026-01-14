#pragma once

#include <cstdint>
#include "tju_local_planning/common/macros.h"
#include "jian/jian.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

class TLog {
 public:
  static void init(const char *log_file_path = "/var/log/tju_local_planning.log",
                   int32_t level = jian::logging::level_enum::trace, int32_t max_file_num = 20);
};

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END

/**
 * @brief log init function, use it when your process started
 * @param log_file_path [const char *] log file saved path, default: "/var/log/port_perception.log"
 * @param level [int32_t] minimum level to logging, default: jian::logging::level_enum::trace
 * @param max_file_num [int32_t] max file number (each file's max size is 1Mb), default: 20
 */
#define LOGINIT(...) TJU_LOCAL_PLANNING_LIB_NAMESPACE::TLog::init(__VA_ARGS__)
/**
 * @brief trace level log, e.g. NTTRACE << "trace info";
 * @ingroup tools_t_log
 */
#define NTTRACE JLOGGER_TRACE_S("local_planning")
/**
 * @brief debug level log, e.g. NTDEBUG << "debug info";
 * @ingroup tools_t_log
 */
#define NTDEBUG JLOGGER_DEBUG_S("local_planning")
/**
 * @brief info level log, e.g. NTINFO << "info info";
 *  @ingroup tools_t_log
 */
#define NTINFO JLOGGER_INFO_S("local_planning")
/**
 * @brief warning level log, e.g. NTWARNING << "info info";
 * @ingroup tools_t_log
 */
#define NTWARNING JLOGGER_WARN_S("local_planning")
/**
 * @brief error level log, e.g. NTERROR << "error info";
 * @ingroup tools_t_log
 */
#define NTERROR JLOGGER_ERROR_S("local_planning")
/**
 * @brief fatal level log, e.g. NTFATAL << "fatal info";
 * @ingroup tools_t_log
 */
#define NTFATAL JLOGGER_CRITICAL_S("local_planning")
