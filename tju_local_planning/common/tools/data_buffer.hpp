#pragma once

#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <set>

#include "tju_local_planning/common/error/code.hpp"
#include "tju_local_planning/common/macros.h"
#include "tju_local_planning/tools/log/t_log.h"

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_BEGIN

template <typename T>
struct SensorData {
  double time;
  T data;

  explicit SensorData(const double& time = 0.0, const T& data = nullptr) : time(time), data(data) {}

  bool operator<(const SensorData& other) const {
    if (std::abs(time - other.time) < EPSILON) {
      return false;
    }
    return time < other.time;
  }
};

/**
 * @brief 数据缓冲区类模板,线程安全
 * @note 后续如果需要添加callback，可以添加条件变量和一个管理回调函数的容器实现. Mark.
 * @tparam DataType 缓冲区存储的数据类型
 */
template <typename DataType>
class DataBuffer {
 private:
  std::set<DataType> buffer_;   ///< 存储数据的集合
  uint32_t buffer_size_;        ///< 缓冲区的最大容量
  double max_time_delay_;       ///< 最大时间延迟，单位秒
  std::string name_;            ///< 缓冲区的名称
  std::mutex mutex_;            ///< 互斥锁
  uint32_t dropped_times_;      ///< 当时间回滚时，丢弃的数据数量
  uint32_t max_dropped_times_;  ///< 最大丢弃次数, 当丢弃次数超过该值时，清空缓冲区

 public:
  /**
   * @brief 构造函数
   *
   * @param buffer_size 缓冲区的最大容量
   * @param name 缓冲区的名称(可选)
   * @param max_time_delay 最大时间延迟(默认为0.5)
   */
  explicit DataBuffer(const uint32_t& buffer_size, const std::string& name = "", const double& max_time_delay = 0.5,
                      const uint32_t& max_dropped_times = 2)
      : buffer_size_(buffer_size),
        max_time_delay_(max_time_delay),
        name_(name),
        max_dropped_times_(max_dropped_times) {}

  /**
   * @brief 析构函数
   */
  ~DataBuffer() = default;

  /**
   * @brief 向缓冲区添加数据
   *
   * @param data 要添加的数据
   */
  uint32_t push(const DataType& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!buffer_.empty() && buffer_.rbegin()->time > data.time) {
      NTWARNING << "Buffer: " << name_ << " roll back. New data time: " << data.time
               << " is earlier than old data time: " << buffer_.rbegin()->time;
      if (++dropped_times_ > max_dropped_times_) {
        buffer_.clear();
        dropped_times_ = 0;
      }
      return ErrorCode::DATA_BUFFER_ROLLBACK;
    }

    if (buffer_.size() >= buffer_size_) {
      buffer_.erase(buffer_.begin());
    }
    buffer_.emplace(data);

    return ErrorCode::SUCCESS;
  }

  /**
   * @brief 从缓冲区移除指定的数据
   *
   * @param data 要移除的数据
   */
  uint32_t extractByTime(const double& time, DataType& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (buffer_.empty()) {
      NTERROR << "Buffer: " << name_ << " is empty.";
      return ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_EMPTY;
    }

    auto it = buffer_.lower_bound(DataType(time));

    if (it == buffer_.end()) {
      // 如果所有数据都早于请求时间
      auto last = std::prev(buffer_.end());
      if (time - last->time > max_time_delay_) {
        NTERROR << "Buffer: " << name_ << " extract failed for timeout. Time: " << time
               << " is later than old data time: " << last->time;
        return ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT;
      }
      data = *last;
    } else if (it == buffer_.begin()) {
      // 如果所有数据都晚于请求时间
      if (it->time - time > max_time_delay_) {
        NTERROR << "Buffer: " << name_ << " extract failed for timeout. Time: " << time
               << " is earlier than earliest data time: " << it->time;
        return ErrorCode::DATA_BUFFER_EXTRACT_FAILED_FOR_TIMEOUT;
      }
      data = *it;
    } else {
      // 找到了两个相邻的数据点
      auto prev = std::prev(it);
      if (it->time - time < time - prev->time) {
        data = *it;
      } else {
        data = *prev;
      }
    }

    return ErrorCode::SUCCESS;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
    dropped_times_ = 0;
  }

  void setMaxTimeDelay(const double& max_time_delay) { max_time_delay_ = max_time_delay; }

  void setMaxDroppedTimes(const uint32_t& max_dropped_times) { max_dropped_times_ = max_dropped_times; }

  void setBufferSize(const uint32_t& buffer_size) { buffer_size_ = buffer_size; }

  const std::string& name() const { return name_; }

  uint32_t size() const { return buffer_.size(); }

  /**
   * @brief 获取最新数据的时间
   *
   * @return double 时间
   */
  double getLatestDataTime() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {  // 如果buffe为空，返回异常值-1.0
      NTERROR << "Buffer: " << name_ << " is empty, getLatestDataTime failed!";
      return -1.0;
    }

    return buffer_.rbegin()->time;
  }

  /**
   * @brief 获取缓冲区数据
   *
   * @return std::set<DataType> 缓冲区数据
   */
  const std::set<DataType> getBuffer() {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_;
  }

  /**
   * @brief 打印缓冲区中数据的起止时间和个数
   */
  void printBuffer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
      std::cout << "Buffer: " << name_ << " is empty." << std::endl;
      return;
    }
    std::cout << "Buffer: " << name_ << std::endl;
    std::cout << "Start time: " << buffer_.begin()->time << std::endl;
    std::cout << "End time: " << buffer_.rbegin()->time << std::endl;
    std::cout << "Size: " << buffer_.size() << std::endl;

    // print all data timestamp
    // for (const auto& data : buffer_) {
    //   printf("Data timestamp: %.4lf\n", data.time);
    // }
  }
};

TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE_END

/**
 * @brief 注册传感器数据类型(将数据用传感器统一接口封装便于统一管理)
 * @tparam DataType 传感器数据类型
 * @note DataType 需要满足以下条件:
 * 1. 指针类型
 */
#define REGISTOR_SENSOR_DATA(DataName, DataType)                                          \
  struct DataName : public TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::SensorData<DataType> { \
    using TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::SensorData<DataType>::SensorData;       \
  };                                                                                      \
  typedef TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::DataBuffer<DataName> DataName##Buffer;  \
  typedef std::shared_ptr<DataName##Buffer> DataName##BufferPtr;
