/**
 * @file t_log.h
 * @author WangXu (xuwang_note@hotmail.com)
 * @brief 日志工具
 * @version 0.1
 * @date 2024-09-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "common/macros.h"
#include "jian/jian.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class TLog {
 public:
  static void init(const char *log_file_path = "/var/log/trunk_perception.log",
                   int32_t level = jian::logging::level_enum::trace, int32_t max_file_num = 20);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END

/**
 * @brief log init function, use it when your process started
 * @param log_file_path [const char *] log file saved path, default: "/var/log/port_perception.log"
 * @param level [int32_t] minimum level to logging, default: jian::logging::level_enum::trace
 * @param max_file_num [int32_t] max file number (each file's max size is 1Mb), default: 20
 */
#define LOGINIT(...) TRUNK_PERCEPTION_LIB_NAMESPACE::TLog::init(__VA_ARGS__)
/**
 * @brief trace level log, e.g. TTRACE << "trace info";
 * @ingroup tools_t_log
 */
#define TTRACE JLOGGER_TRACE_S("perception")
/**
 * @brief debug level log, e.g. TDEBUG << "debug info";
 * @ingroup tools_t_log
 */
#define TDEBUG JLOGGER_DEBUG_S("perception")
/**
 * @brief info level log, e.g. TINFO << "info info";
 *  @ingroup tools_t_log
 */
#define TINFO JLOGGER_INFO_S("perception")
/**
 * @brief warning level log, e.g. TWARNING << "info info";
 * @ingroup tools_t_log
 */
#define TWARNING JLOGGER_WARN_S("perception")
/**
 * @brief error level log, e.g. TERROR << "error info";
 * @ingroup tools_t_log
 */
#define TERROR JLOGGER_ERROR_S("perception")
/**
 * @brief fatal level log, e.g. TFATAL << "fatal info";
 * @ingroup tools_t_log
 */
#define TFATAL JLOGGER_CRITICAL_S("perception")