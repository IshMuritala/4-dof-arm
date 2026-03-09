#ifndef INC_4DOF_ARM_TRACKING_H
#define INC_4DOF_ARM_TRACKING_H

#include <string>
#include <vector>
#include "grasp_system/perception/features_color.h"

enum class TrackState {
    NEW,
    TRACKED,
    LOST
};

struct TrackedObject {
    std::string id;
    int id_num = -1;
    bool id_assigned = false;
    ClusterFeatures features;
    TrackState state = TrackState::NEW;
    int seen_streak = 0;
    int total_seen = 0;
    int lost_count = 0;
    bool seen_this_frame = false;
    int last_det_idx = -1;
    bool has_smoothed = false;
    Eigen::Vector3f centroid_smoothed = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min_smoothed = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max_smoothed = Eigen::Vector3f::Zero();
    uint8_t color_r = 160;
    uint8_t color_g = 160;
    uint8_t color_b = 160;
};

struct TrackingParams {
    float max_match_distance_m = 0.05f; // Default: 0.05f
    int min_seen_frames = 5;
    int max_missed_frames = 1; // overridden to 3 in test_segmentation
    float smoothing_alpha = 0.3f; // 0.0 -> frozen, 1.0 -> raw
    bool verbose = false;
};

class ObjectTracker {
public:
    explicit ObjectTracker(const TrackingParams& params = TrackingParams());

    void reset();
    std::vector<TrackedObject> update(const std::vector<ClusterFeatures>& detections);

private:
    TrackingParams params_;
    int next_id_ = 0;
    std::vector<TrackedObject> tracks_;
};

#endif //INC_4DOF_ARM_TRACKING_H
