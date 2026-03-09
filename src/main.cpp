#include <filesystem>
#include <iostream>
#include <string>

#include "grasp_system/app/vision_viewer.h"
#include "grasp_system/planning/calibration.h"

namespace {

static std::filesystem::path findProjectRoot() {
    std::filesystem::path dir = std::filesystem::current_path();
    for (int i = 0; i < 6; ++i) {
        if (std::filesystem::exists(dir / "config" / "robot.yaml")) {
            return dir;
        }
        if (!dir.has_parent_path()) {
            break;
        }
        dir = dir.parent_path();
    }
    return {};
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path root = findProjectRoot();
    if (root.empty()) {
        std::cerr << "Could not locate project root (missing config/robot.yaml). "
                     "Run from the repo root or cmake-build-debug." << std::endl;
        return 1;
    }

    const std::string cameraConfigPath = (root / "config" / "camera.yaml").string();
    const std::string calibrationPath = (root / "config" / "calibration.yaml").string();
    const std::string robotConfigPath = (root / "config" / "robot.yaml").string();
    const std::string primitivesPath = (root / "config" / "grasp_primitives.yaml").string();

    Calibration calib = loadCalibration(calibrationPath);
    if (!calib.valid) {
        std::cerr << "Warning: calibration.yaml missing or invalid; points stay in camera frame." << std::endl;
    } else {
        std::cout << "Loaded calibration: RMS=" << calib.rms_error_m
                  << " m, points=" << calib.num_points << std::endl;
    }

    VisionViewerOptions options;
    options.camera_config_path = (argc > 1 && argv[1]) ? argv[1] : cameraConfigPath;
    options.window_title = "4DOF Main";
    options.print_params = true;
    options.show_boxes_default = true;
    options.print_feature_stats = false;
    options.show_calibration_status = true;
    options.calibration = calib.valid ? &calib : nullptr;
    options.enable_pick_preview = true;
    options.enable_pick_execute = true;
    options.robot_config_path = robotConfigPath;
    options.primitives_config_path = primitivesPath;

    return runVisionViewer(options);
}
