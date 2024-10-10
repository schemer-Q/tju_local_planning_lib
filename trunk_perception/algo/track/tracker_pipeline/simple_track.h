/**
 * @file simple_track.h
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "trunk_perception/algo/track/common/geometric_algo.h"
#include "trunk_perception/algo/track/common/id_manager.h"
#include "trunk_perception/algo/track/common/tracklet.h"
#include "trunk_perception/algo/track/matcher/matcher_base.h"
#include "trunk_perception/algo/track/tracker_pipeline/tracker_pipeline_interface.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

struct SimpleTrackParams {
  int min_lifetime_output = 3;
  int max_consecutive_lost_num = 5;
  int min_consecutive_valid_num = 3;
  std::string matcher_method = "";
  std::string traker_method = "";
  YAML::Node traker_params;
};

class SimpleTrack : virtual public TrackerPipelineInterface {
 public:
  SimpleTrack() = default;
  ~SimpleTrack() override = default;

  /**
   * @brief lidar tracker interface init
   *
   * @param config yaml node
   * @return int
   */
  int Init(const YAML::Node& config) override;

  /**
   * @brief track pipeline
   *
   * @param frame data frame
   * @return int
   */
  int Track(std::shared_ptr<OdLidarFrame>& frame) override;

  /**
   * @brief set id manager
   *
   * @param id_manager_ptr tracker id manager pointer
   */
  inline void SetIDManager(const IDManagerPtr& id_manager_ptr) override { id_manager_ptr_ = id_manager_ptr; }

 private:
  /**
   * @brief detection objects preprocess
   *
   * @param objects detection objects
   */
  void preprocess(std::vector<Object>& objects);

  /**
   * @brief transform tracker parameters from previous frame to current frame
   *
   * @param tf
   */
  void transformToCurrentFrame(const Eigen::Isometry3f& tf);

  /**
   * @brief match objects tracked and objects detected
   *
   * @param objects_tracked tracking objects
   * @param objects_detected detection objects
   * @param assignments pair assigned tracking objects and detection objects
   * @param unassigned_tracks unassigned tracking objects
   * @param unassigned_objects unassigned detection objects
   */
  void match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
             std::vector<TrackObjectPair>& assignments, std::vector<size_t>& unassigned_tracks,
             std::vector<size_t>& unassigned_objects);

  /**
   * @brief update Assigned tracks with objects detected
   *
   * @param objects_detected current frame objects detected
   * @param assignments pair assigned tracking objects and detection objects
   */
  void updateAssignedTracks(const std::vector<Object>& objects_detected,
                            const std::vector<TrackObjectPair>& assignments);

  /**
   * @brief update unmatched tracks
   *
   * @param unassigned_tracks unassigned tracking objects index
   * @param timestamp current frame timestamp
   */
  void updateUnassignedTracks(const std::vector<size_t>& unassigned_tracks, const double timestamp);

  /**
   * @brief push unassigned objects to tracks
   *
   * @param objects_detected current frame objects detected
   * @param unassigned_objects unassigned tracking objects index
   */
  void pushNewTracks(const std::vector<Object>& objects_detected, const std::vector<size_t>& unassigned_objects);

  /**
   * @brief add detection object to tracklet
   *
   * @param object_detected current frame detection object
   */
  void addObjectToTrack(const Object& object_detected);

  /**
   * @brief manager track object life cycle
   *
   */
  void managerLifeCycle();

  /**
   * @brief nms to remove overlap tracking object
   *
   */
  void nms();

  /**
   * @brief output track objects result
   *
   * @param tracked_objects tracked objects
   */
  void outputTrackResult(std::vector<Object>& tracked_objects);

 private:
  SimpleTrackParams params_;                            // track param
  IDManagerPtr id_manager_ptr_ = nullptr;               // ID manager
  std::vector<Tracklet> tracklets_;                     // tracking object list
  std::unique_ptr<MatcherBase> matcher_ptr_ = nullptr;  // matcher method
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END