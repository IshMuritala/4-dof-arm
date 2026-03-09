#ifndef INC_4DOF_ARM_SCENE_SUMMARY_H
#define INC_4DOF_ARM_SCENE_SUMMARY_H

#include <string>
#include <vector>
#include "grasp_system/perception/features_color.h"
#include "grasp_system/perception/tracking.h"

namespace cv {
class Mat;
}


struct ObjectInfo {
    std::string id;
    ClusterFeatures features;
    TrackState state = TrackState::TRACKED;
    bool seen_this_frame = true;
    bool id_assigned = true;
};


struct SceneSummary {
    std::vector<ObjectInfo> objects;
    bool valid = false;
};

struct SceneSummaryOptions {
    bool require_assigned_id = true;
    bool require_seen_this_frame = true;
    bool include_lost = false;
    bool use_smoothed = true;
};

struct SceneSummaryConfig {
    bool debug = false;
    std::string log_path = "logs/scene_summary.log";
    std::string jsonl_path = "logs/scene_summary.jsonl";
    std::string annotated_dir = "logs/annotated";
    bool include_new_in_log = true;
    bool include_lost_in_log = true;
    bool print_to_console = true;
    double print_interval_sec = 2.0;
};

struct CameraIntrinsics {
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int width = 0;
    int height = 0;
};


SceneSummary buildSceneSummary(const std::vector<ClusterFeatures>& features,
                               const std::string& id_prefix = "obj");


SceneSummary buildSceneSummary(const std::vector<TrackedObject>& tracks,
                               const SceneSummaryOptions& options = SceneSummaryOptions());


std::string formatSceneSummary(const SceneSummary& summary, int precision = 3);


std::string formatSceneSummaryJson(const SceneSummary& summary, int precision = 3);


bool appendSceneSummaryLog(const SceneSummary& summary, const std::string& path);


bool appendSceneSummaryJsonl(const SceneSummary& summary, const std::string& path);


SceneSummaryConfig loadSceneSummaryConfig(const std::string& yamlPath);


bool annotateSceneSummary(const SceneSummary& summary,
                          const cv::Mat& colorBgr,
                          const CameraIntrinsics& intr,
                          cv::Mat& outAnnotated);


bool saveAnnotatedSceneFrame(const SceneSummary& summary,
                             const cv::Mat& colorBgr,
                             const CameraIntrinsics& intr,
                             const std::string& outPath);

#endif //INC_4DOF_ARM_SCENE_SUMMARY_H
