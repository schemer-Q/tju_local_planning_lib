#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_NAMESPACE_BEGIN

void TLog::init(const char *log_file_path, int32_t level, int32_t max_file_num) {
  // jian::logging::global_config()
  //     .level(static_cast<jian::logging::level_enum>(level))  // 设置日志等级
  //     .filename(log_file_path)                               // 设置要存储的目录以及文件名称
  //     .filesize(1 << 20)                                     // 设置单个文件最大为1<<20字节, 即1MB
  //     .filerotate(max_file_num)                              // 循环存储, 设置最多保存多少个文件
  //     .async()
  //     .emit();                                               // 提交生效

  auto my_term_sink = jian::logging::create_sink<jian::logging::sinks::stdout_color_sink_st>();
  auto my_file_sink = jian::logging::create_sink<jian::logging::sinks::rotating_dispatch_files_sink_mt>(
      log_file_path, 1 << 20, max_file_num);
  auto my_logger = jian::logging::register_logger("local_planning", {my_term_sink, my_file_sink});
  my_logger->set_level(static_cast<jian::logging::level_enum>(level));
  my_logger->flush_on(jian::logging::level_enum::err);
}

TJU_LOCAL_PLANNING_LIB_NAMESPACE_END
