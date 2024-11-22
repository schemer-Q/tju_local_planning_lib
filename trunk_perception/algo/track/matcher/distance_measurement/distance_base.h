/**
 * @file distance_measurement_base.h
 * @author Fan Dongsheng (fandongsheng@trunk.tech)
 * @brief
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <yaml-cpp/yaml.h>

#include "trunk_perception/algo/track/common/tracklet.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

class DistanceBase {
 public:
  DistanceBase() = default;
  virtual ~DistanceBase() = default;

  /**
   * @brief object distance measurement init
   *
   * @param config yaml node
   * @return int
   */
  virtual int Init(const YAML::Node& config) = 0;

  /**
   * @brief compute track and object measurement distance
   *
   * @param track tracking object
   * @param object current detection object
   * @return float measurement distance
   */
  virtual float ComputeDistance(const Tracklet& track, const Object& object) = 0;
};

class DistanceRegistry {
 public:
  static DistanceRegistry& Get() {
    static DistanceRegistry instance;
    return instance;
  }

  void Register(const std::string& name, std::function<std::unique_ptr<DistanceBase>()> matcher) {
    factories[name] = std::move(matcher);
  }

  std::unique_ptr<DistanceBase> Create(const std::string& name) {
    if (factories.find(name) == factories.end()) {
      return nullptr;
    }
    return factories[name]();
  }

 private:
  std::unordered_map<std::string, std::function<std::unique_ptr<DistanceBase>()>> factories;
};

#define REGISTER_DISTANCE(NAME, TYPE)                                                                \
  struct Register##TYPE {                                                                            \
   public:                                                                                           \
    Register##TYPE() {                                                                               \
      auto lambda_func = []() -> std::unique_ptr<DistanceBase> { return std::make_unique<TYPE>(); }; \
      DistanceRegistry::Get().Register(NAME, lambda_func);                                           \
    }                                                                                                \
  };                                                                                                 \
  static Register##TYPE staticRegister##TYPE;

TRUNK_PERCEPTION_LIB_NAMESPACE_END