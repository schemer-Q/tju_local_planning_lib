/**
 * @file simple_track.cpp
 * @author Fan Dongsheng
 * @brief
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <numeric>

#include "trunk_perception/algo/track/tracker_method/tracker_method_base.h"
#include "trunk_perception/algo/track/tracker_pipeline/simple_track.h"
#include "trunk_perception/tools/log/t_log.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

int SimpleTrack::Init(const YAML::Node& config) {
  try {
    // tracker pipeline param init
    params_.min_lifetime_output = config["min_lifetime_output"].as<int>();
    params_.max_consecutive_lost_num = config["max_consecutive_lost_num"].as<int>();
    params_.min_consecutive_valid_num = config["min_consecutive_valid_num"].as<int>();

    // ID manager init
    if (config["IDManager"].IsDefined()) {
      id_manager_ptr_ = std::make_shared<IDManager>();
      if (id_manager_ptr_) {
        id_manager_ptr_->Init(config["IDManager"]);
      }
    }

    // matcher init
    params_.matcher_method = config["MatcherMethod"].as<std::string>();
    matcher_ptr_ = MatcherRegistry::Get().Create(params_.matcher_method);
    if (!matcher_ptr_) {
      TFATAL << "[SimpleTrack.Init] matcher_ptr_ is nullptr";
      return 1;
    }
    matcher_ptr_->Init(config["MatcherParams"]);

    // tracker method param
    params_.traker_method = config["TrackerMethod"].as<std::string>();
    params_.traker_params = config["TrackerParams"];
  } catch (const std::exception& e) {
    TFATAL << "[SimpleTrack] LoadYAMLConfig failed! " << e.what();
    return 1;
  }

  return 0;
}

int SimpleTrack::Track(std::shared_ptr<OdLidarFrame>& frame) {
  // detection objects preprocess
  preprocess(frame->detected_objects);
  const auto& objects_detected = frame->detected_objects;

  // Transform to current frame
  transformToCurrentFrame(frame->tf);

  // match detection and track
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  match(tracklets_, objects_detected, assignments, unassigned_tracks, unassigned_objects);

  // update assigned objects tracked
  updateAssignedTracks(objects_detected, assignments);

  // update unassigned objects tracked
  updateUnassignedTracks(unassigned_tracks, frame->timestamp);

  // push unassigned objects to tracks
  pushNewTracks(objects_detected, unassigned_objects);

  // manager track life cycle
  managerLifeCycle();

  // output track result
  outputTrackResult(frame->tracked_objects);

  return 0;
}

void SimpleTrack::preprocess(std::vector<Object>& objects) {
  for (auto& object : objects) {
    auto& bbox = object.bbox;

    // 按距离从最近点重新排列角点
    int nearest_id = 0;
    bbox.corners2d.colwise().squaredNorm().minCoeff(&nearest_id);
    if (nearest_id != 0) {
      const auto temp = bbox.corners2d;
      const int sz = bbox.corners2d.cols();
      for (int i = 0; i < sz; ++i) {
        bbox.corners2d.col(i) = temp.col((nearest_id + i) % sz);
      }
    }

    // 计算bbox航向向量
    object.bbox.direction << std::cos(object.bbox.theta), std::sin(object.bbox.theta), 0.0f;

    // 计算LShape feature
    computeLShapeFeature(object.bbox, object.l_shape_feature);

    // output tracking point to display
    object.track_point = Eigen::Vector3f::Zero();
    if (params_.traker_method == "NearestCornerTrackerCV") {
      object.track_point.head(2) = object.l_shape_feature.reference_point.cast<float>();
    } else {
      TFATAL << "[SimpleTrack] traker_method is error!";
      return;
    }
  }
}

void SimpleTrack::transformToCurrentFrame(const Eigen::Isometry3f& tf) {
  for (auto& tracklet : tracklets_) {
    tracklet.TransformToCurrent(tf);
  }
}

void SimpleTrack::match(const std::vector<Tracklet>& objects_tracked, const std::vector<Object>& objects_detected,
                        std::vector<TrackObjectPair>& assignments, std::vector<size_t>& unassigned_tracks,
                        std::vector<size_t>& unassigned_objects) {
  assignments.clear();
  unassigned_objects.clear();
  unassigned_tracks.clear();

  if (objects_tracked.empty() || objects_detected.empty()) {
    unassigned_tracks.resize(objects_tracked.size());
    unassigned_objects.resize(objects_detected.size());
    std::iota(unassigned_tracks.begin(), unassigned_tracks.end(), 0);
    std::iota(unassigned_objects.begin(), unassigned_objects.end(), 0);
    return;
  }

  matcher_ptr_->Match(objects_tracked, objects_detected, &assignments, &unassigned_tracks, &unassigned_objects);
}

void SimpleTrack::updateAssignedTracks(const std::vector<Object>& objects_detected,
                                       const std::vector<TrackObjectPair>& assignments) {
  for (size_t i = 0UL; i < assignments.size(); ++i) {
    const auto& object = objects_detected[assignments[i].second];
    auto& track = tracklets_[assignments[i].first];
    track.Update(object);

    if (track.current_tracking_object.lifetime >= params_.min_consecutive_valid_num &&
        track.state == TrackletState::UNCONFIRMED) {
      track.state = TrackletState::CONFIRMED;
    }
  }
}

void SimpleTrack::updateUnassignedTracks(const std::vector<size_t>& unassigned_tracks, const double timestamp) {
  for (size_t i = 0UL; i < unassigned_tracks.size(); ++i) {
    auto& track = tracklets_[unassigned_tracks[i]];
    track.Predict(timestamp);

    if (track.current_tracking_object.consecutive_lost >= params_.max_consecutive_lost_num ||
        track.state == TrackletState::UNCONFIRMED) {
      track.state = TrackletState::DEAD;
    }
  }
}

void SimpleTrack::pushNewTracks(const std::vector<Object>& objects_detected,
                                const std::vector<size_t>& unassigned_objects) {
  for (size_t i = 0UL; i < unassigned_objects.size(); ++i) {
    addObjectToTrack(objects_detected[unassigned_objects[i]]);
  }
}

void SimpleTrack::addObjectToTrack(const Object& object_detected) {
  if (!id_manager_ptr_) {
    TFATAL << "[SimpleTrack] id manager ptr is nullptr";
    return;
  }

  Tracklet track;
  track.current_tracking_object = object_detected;
  track.current_tracking_object.track_id = id_manager_ptr_->ExtractID();
  track.current_tracking_object.lifetime = 1UL;
  track.state = TrackletState::UNCONFIRMED;
  track.tracker_method_ptr = TrackerMethodRegistry::Get().Create(params_.traker_method);
  if (!track.tracker_method_ptr) {
    TFATAL << "[SimpleTrack.addObjectToTrack] track.tracker_method_ptr is nullptr";
    return;
  }
  track.tracker_method_ptr->Init(params_.traker_params, object_detected);
  tracklets_.emplace_back(track);
}

void SimpleTrack::managerLifeCycle() {
  if (!id_manager_ptr_) {
    TFATAL << "[SimpleTrack] id manager ptr is nullptr";
    return;
  }

  if (tracklets_.empty()) return;

  // nms
  nms();

  // recycle track id
  for (const auto& track : tracklets_) {
    if (track.Dieout()) {
      id_manager_ptr_->RecycleID(track.current_tracking_object.track_id);
    }
  }

  // manager track life cycle
  auto con = [](const Tracklet& track) { return track.Dieout(); };
  tracklets_.erase(std::remove_if(tracklets_.begin(), tracklets_.end(), con), tracklets_.end());
}

void SimpleTrack::nms() {
  const size_t size = tracklets_.size();
  for (size_t i = 0UL; i < size; ++i) {
    auto& tracked_i = tracklets_[i];
    if (tracked_i.Dieout()) continue;

    for (size_t j = i + 1UL; j < size; ++j) {
      auto& tracked_j = tracklets_[j];
      if (tracked_j.Dieout()) continue;

      auto& object_i = tracked_i.current_tracking_object;
      auto& object_j = tracked_j.current_tracking_object;
      const double iou = getOverlapRate(object_i.bbox.corners2d, object_j.bbox.corners2d);
      if (iou > 0.3) {
        if (object_j.lifetime > object_i.lifetime) {
          tracked_i.state = TrackletState::DEAD;
        } else {
          tracked_j.state = TrackletState::DEAD;
        }
      }
    }
  }
}

void SimpleTrack::outputTrackResult(std::vector<Object>& tracked_objects) {
  tracked_objects.clear();
  for (const auto& track : tracklets_) {
    if (track.current_tracking_object.lifetime >= params_.min_lifetime_output &&
        track.state == TrackletState::CONFIRMED) {
      tracked_objects.emplace_back(track.current_tracking_object);
    }
  }
}

TRUNK_PERCEPTION_LIB_NAMESPACE_END