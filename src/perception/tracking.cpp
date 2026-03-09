#include "grasp_system/perception/tracking.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {
struct MatchPair {
    int track_idx;
    int det_idx;
    float dist;
};

float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).norm();
}

const std::array<std::array<uint8_t, 3>, 12> kPalette = {{
    {{255,  80,  80}},
    {{ 80, 200, 255}},
    {{120, 255, 120}},
    {{255, 200,  80}},
    {{200, 120, 255}},
    {{255, 120, 200}},
    {{120, 200, 120}},
    {{200, 200,  80}},
    {{ 80, 120, 255}},
    {{255, 150,  60}},
    {{ 60, 220, 180}},
    {{180,  60, 220}}
}};

std::array<uint8_t, 3> pickColor(int id) {
    std::size_t idx = static_cast<std::size_t>(id) % kPalette.size();
    return kPalette[idx];
}

} // namespace

ObjectTracker::ObjectTracker(const TrackingParams& params) : params_(params) {}

void ObjectTracker::reset() {
    tracks_.clear();
    next_id_ = 0;
}

std::vector<TrackedObject> ObjectTracker::update(const std::vector<ClusterFeatures>& detections) {
    std::vector<TrackedObject> output;

    for (auto& track : tracks_) {
        track.seen_this_frame = false;
        track.last_det_idx = -1;
    }

    std::vector<MatchPair> pairs;
    pairs.reserve(tracks_.size() * detections.size());

    for (int t = 0; t < static_cast<int>(tracks_.size()); ++t) {
        for (int d = 0; d < static_cast<int>(detections.size()); ++d) {
            if (!detections[d].valid) {
                continue;
            }
            const Eigen::Vector3f& tcentroid = tracks_[t].has_smoothed ?
                                               tracks_[t].centroid_smoothed :
                                               tracks_[t].features.centroid;
            float dist = distance(tcentroid, detections[d].centroid);
            if (dist <= params_.max_match_distance_m) {
                pairs.push_back({t, d, dist});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const MatchPair& a, const MatchPair& b) { return a.dist < b.dist; });

    std::vector<bool> trackMatched(tracks_.size(), false);
    std::vector<bool> detMatched(detections.size(), false);

    for (const auto& pair : pairs) {
        if (trackMatched[pair.track_idx] || detMatched[pair.det_idx]) {
            continue;
        }
        trackMatched[pair.track_idx] = true;
        detMatched[pair.det_idx] = true;

        auto& track = tracks_[pair.track_idx];
        track.features = detections[pair.det_idx];
        track.total_seen += 1;
        track.seen_streak += 1;
        track.lost_count = 0;
        track.seen_this_frame = true;
        track.last_det_idx = pair.det_idx;
        track.state = (track.seen_streak >= params_.min_seen_frames) ?
                      TrackState::TRACKED : TrackState::NEW;
        if (!track.id_assigned && track.seen_streak >= params_.min_seen_frames) {
            track.id_num = next_id_++;
            track.id = "obj_" + std::to_string(track.id_num);
            auto color = pickColor(track.id_num);
            track.color_r = color[0];
            track.color_g = color[1];
            track.color_b = color[2];
            track.id_assigned = true;
        }

        if (!track.has_smoothed) {
            track.centroid_smoothed = track.features.centroid;
            track.bbox_min_smoothed = track.features.bbox_min;
            track.bbox_max_smoothed = track.features.bbox_max;
            track.has_smoothed = true;
        } else {
            float a = std::min(std::max(params_.smoothing_alpha, 0.0f), 1.0f);
            track.centroid_smoothed = a * track.features.centroid + (1.0f - a) * track.centroid_smoothed;
            track.bbox_min_smoothed = a * track.features.bbox_min + (1.0f - a) * track.bbox_min_smoothed;
            track.bbox_max_smoothed = a * track.features.bbox_max + (1.0f - a) * track.bbox_max_smoothed;
        }
    }


    std::vector<TrackedObject> kept;
    kept.reserve(tracks_.size());
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
        auto& track = tracks_[t];
        if (trackMatched[t]) {
            kept.push_back(track);
            continue;
        }
        track.lost_count += 1;
        track.seen_streak = 0;
        if (track.total_seen <= 1 && track.lost_count >= 1) {
            // Ignore one-frame ghosts.
            continue;
        }
        if (track.lost_count > params_.max_missed_frames) {
            continue;
        }
        if (track.lost_count >= params_.max_missed_frames) {
            track.state = TrackState::LOST;
        } else {
            track.state = TrackState::TRACKED;
        }
        kept.push_back(track);
    }
    tracks_.swap(kept);


    for (int d = 0; d < static_cast<int>(detections.size()); ++d) {
        if (detMatched[d] || !detections[d].valid) {
            continue;
        }
        TrackedObject track;
        track.id_num = -1;
        track.id = "obj_pending";
        track.features = detections[d];
        track.total_seen = 1;
        track.seen_streak = 1;
        track.lost_count = 0;
        track.seen_this_frame = true;
        track.last_det_idx = d;
        track.state = (track.seen_streak >= params_.min_seen_frames) ?
                      TrackState::TRACKED : TrackState::NEW;
        track.id_assigned = false;
        track.has_smoothed = true;
        track.centroid_smoothed = track.features.centroid;
        track.bbox_min_smoothed = track.features.bbox_min;
        track.bbox_max_smoothed = track.features.bbox_max;
        tracks_.push_back(track);
    }

    if (params_.verbose) {
        int tracked = 0;
        int lost = 0;
        int news = 0;
        for (const auto& t : tracks_) {
            if (t.state == TrackState::TRACKED) tracked++;
            else if (t.state == TrackState::LOST) lost++;
            else news++;
        }
        std::cout << "tracker: tracked=" << tracked
                  << " new=" << news
                  << " lost=" << lost
                  << std::endl;
    }

    output = tracks_;
    return output;
}
