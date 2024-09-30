/**
 * @file id_manager.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-08-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <deque>

#include "trunk_perception/common/macros.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class IDManager {
 public:
  IDManager() { initIDPool(); }

  IDManager(const size_t start_id, const size_t max_id) : start_id_(start_id), max_id_(max_id) { initIDPool(); }

  ~IDManager() = default;

  /**
   * @brief tracker id manager init
   *
   * @param config yaml config node
   * @return int
   */
  int Init(const YAML::Node& config) {
    try {
      start_id_ = config["start_id"].as<size_t>();
      max_id_ = config["max_id"].as<size_t>();
    } catch (const std::exception& e) {
      TFATAL << "[SimpleTrack] LoadYAMLConfig failed! " << e.what();
      return 1;
    }

    initIDPool();

    return 0;
  }

  /**
   * @brief extract tracker id from id pool
   *
   * @return size_t
   */
  size_t ExtractID() {
    if (id_pool_.empty()) {
      TERROR << "[IDManager] id pool is empty()!";
      return max_id_;
    }

    const size_t id = id_pool_.front();
    id_pool_.pop_front();
    return id;
  }

  /**
   * @brief recycle tracker id to id pool
   *
   * @param id
   */
  void RecycleID(const size_t id) {
    if (id < start_id_) return;
    id_pool_.emplace_back(id);
  }

 private:
  void initIDPool() {
    id_pool_.clear();
    for (size_t i = start_id_; i < max_id_; ++i) {
      id_pool_.emplace_back(i);
    }
  }

 private:
  std::deque<size_t> id_pool_;
  size_t start_id_ = 1UL;
  size_t max_id_ = 10000UL;
};

using IDManagerPtr = std::shared_ptr<IDManager>;
using IDManagerConstPtr = std::shared_ptr<const IDManager>;

TRUNK_PERCEPTION_LIB_NAMESPACE_END