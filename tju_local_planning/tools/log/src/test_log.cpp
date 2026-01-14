#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "tju_local_planning/tools/log/t_log.h"

int main(int argc, char* argv[]) {
    // 初始化日志系统
    // 参数: 日志文件路径, 日志级别, 日志文件数量上限
    LOGINIT("./log_demo.log", jian::logging::level_enum::trace, 5);
    
    std::cerr << "日志系统已初始化，日志文件路径: ./log_demo.log" << std::endl;
    
    // // 测试不同级别的日志
    NTTRACE << "这是一条跟踪日志，包含详细的程序执行信息";
    NTDEBUG << "这是一条调试日志，通常用于开发调试阶段";
    NTINFO << "这是一条信息日志，记录程序正常运行的状态信息";
    NTWARNING << "这是一条警告日志，表示可能存在的问题，但不影响正常运行";
    NTERROR << "这是一条错误日志，表示发生了错误但程序仍可继续运行";
    // NTFATAL << "这是一条致命错误日志，表示严重问题可能导致程序崩溃或终止"; //此命令会终止程序 即使程序没错
    
    // // 演示日志中包含变量
    int count = 42;
    std::string message = "Hello, Logging!";
    double value = 3.14159;
    
    NTINFO << "变量示例 - 计数: " << count << ", 消息: " << message << ", 值: " << value;
       
    return 0;
}