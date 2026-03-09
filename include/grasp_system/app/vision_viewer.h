

#ifndef INC_4DOF_ARM_VISION_VIEWER_H
#define INC_4DOF_ARM_VISION_VIEWER_H

#include <string>

struct Calibration;

struct VisionViewerOptions {
    std::string camera_config_path = "config/camera.yaml";
    std::string window_title = "Vision Viewer";
    bool print_params = true;
    bool show_boxes_default = false;
    bool print_feature_stats = false;
    bool show_calibration_status = false;
    const Calibration* calibration = nullptr;
    // pick preview (no motion). Uses bbox width -> gripper mapping
    bool enable_pick_preview = false;
    // pick execution (sends motion to Arduino)
    bool enable_pick_execute = false;
    std::string robot_config_path = "config/robot.yaml";
    std::string primitives_config_path = "config/grasp_primitives.yaml";
};


int runVisionViewer(const VisionViewerOptions& options);

#endif //INC_4DOF_ARM_VISION_VIEWER_H
