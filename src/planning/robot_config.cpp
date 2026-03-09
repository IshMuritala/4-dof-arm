#include "grasp_system/planning/robot_config.h"

#include <opencv2/core.hpp>
#include <iostream>

namespace {

template <typename T>
bool readNodeIfPresent(const cv::FileNode& node, const char* key, T& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> value;
        return true;
    }
    return false;
}

void warnMissing(const std::string& yamlPath, const std::string& keyPath) {
    std::cerr << "loadRobotConfig: missing '" << keyPath
              << "' in " << yamlPath << " (using default)" << std::endl;
}

} // namespace

RobotConfig loadRobotConfig(const std::string& yamlPath) {
    RobotConfig cfg;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "loadRobotConfig: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return cfg;
    }
    if (!fs.isOpened()) {
        return cfg;
    }

    const cv::FileNode root = fs["robot"];
    if (root.empty()) {
        std::cerr << "loadRobotConfig: missing 'robot' root in " << yamlPath
                  << " (using defaults)" << std::endl;
        return cfg;
    }

    const cv::FileNode geometry = root["geometry"];
    if (!geometry.empty()) {
        if (!readNodeIfPresent(geometry, "base_height_m", cfg.geometry.base_height_m)) {
            warnMissing(yamlPath, "robot.geometry.base_height_m");
        }
        if (!readNodeIfPresent(geometry, "shoulder_to_elbow_m", cfg.geometry.shoulder_to_elbow_m)) {
            warnMissing(yamlPath, "robot.geometry.shoulder_to_elbow_m");
        }
        if (!readNodeIfPresent(geometry, "elbow_to_wrist_m", cfg.geometry.elbow_to_wrist_m)) {
            warnMissing(yamlPath, "robot.geometry.elbow_to_wrist_m");
        }
        if (!readNodeIfPresent(geometry, "wrist_to_gripper_forward_m", cfg.geometry.wrist_to_gripper_forward_m)) {
            warnMissing(yamlPath, "robot.geometry.wrist_to_gripper_forward_m");
        }
        if (!readNodeIfPresent(geometry, "wrist_to_gripper_up_m", cfg.geometry.wrist_to_gripper_up_m)) {
            warnMissing(yamlPath, "robot.geometry.wrist_to_gripper_up_m");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.geometry' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode limits = root["joint_limits_deg"];
    if (!limits.empty()) {
        if (!readNodeIfPresent(limits, "base_min", cfg.joint_limits.base_min)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.base_min");
        }
        if (!readNodeIfPresent(limits, "base_max", cfg.joint_limits.base_max)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.base_max");
        }
        if (!readNodeIfPresent(limits, "shoulder_min", cfg.joint_limits.shoulder_min)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.shoulder_min");
        }
        if (!readNodeIfPresent(limits, "shoulder_max", cfg.joint_limits.shoulder_max)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.shoulder_max");
        }
        if (!readNodeIfPresent(limits, "elbow_min", cfg.joint_limits.elbow_min)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.elbow_min");
        }
        if (!readNodeIfPresent(limits, "elbow_max", cfg.joint_limits.elbow_max)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.elbow_max");
        }
        if (!readNodeIfPresent(limits, "wrist_min", cfg.joint_limits.wrist_min)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.wrist_min");
        }
        if (!readNodeIfPresent(limits, "wrist_max", cfg.joint_limits.wrist_max)) {
            warnMissing(yamlPath, "robot.joint_limits_deg.wrist_max");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.joint_limits_deg' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode home = root["home_pose_deg"];
    if (!home.empty()) {
        if (!readNodeIfPresent(home, "base", cfg.home.base)) {
            warnMissing(yamlPath, "robot.home_pose_deg.base");
        }
        if (!readNodeIfPresent(home, "shoulder", cfg.home.shoulder)) {
            warnMissing(yamlPath, "robot.home_pose_deg.shoulder");
        }
        if (!readNodeIfPresent(home, "elbow", cfg.home.elbow)) {
            warnMissing(yamlPath, "robot.home_pose_deg.elbow");
        }
        if (!readNodeIfPresent(home, "wrist", cfg.home.wrist)) {
            warnMissing(yamlPath, "robot.home_pose_deg.wrist");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.home_pose_deg' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode offsets = root["joint_offsets_deg"];
    if (!offsets.empty()) {
        if (!readNodeIfPresent(offsets, "base", cfg.joint_offsets.base)) {
            warnMissing(yamlPath, "robot.joint_offsets_deg.base");
        }
        if (!readNodeIfPresent(offsets, "shoulder", cfg.joint_offsets.shoulder)) {
            warnMissing(yamlPath, "robot.joint_offsets_deg.shoulder");
        }
        if (!readNodeIfPresent(offsets, "elbow", cfg.joint_offsets.elbow)) {
            warnMissing(yamlPath, "robot.joint_offsets_deg.elbow");
        }
        if (!readNodeIfPresent(offsets, "wrist", cfg.joint_offsets.wrist)) {
            warnMissing(yamlPath, "robot.joint_offsets_deg.wrist");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.joint_offsets_deg' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode gripper = root["gripper"];
    if (!gripper.empty()) {
        if (!readNodeIfPresent(gripper, "open", cfg.gripper.open)) {
            warnMissing(yamlPath, "robot.gripper.open");
        }
        if (!readNodeIfPresent(gripper, "closed", cfg.gripper.closed)) {
            warnMissing(yamlPath, "robot.gripper.closed");
        }
        if (!readNodeIfPresent(gripper, "home", cfg.gripper.home)) {
            warnMissing(yamlPath, "robot.gripper.home");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.gripper' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode motion = root["motion_limits"];
    if (!motion.empty()) {
        if (!readNodeIfPresent(motion, "joint_max_deg_per_s", cfg.motion.joint_max_deg_per_s)) {
            warnMissing(yamlPath, "robot.motion_limits.joint_max_deg_per_s");
        }
        if (!readNodeIfPresent(motion, "joint_max_deg_per_s2", cfg.motion.joint_max_deg_per_s2)) {
            warnMissing(yamlPath, "robot.motion_limits.joint_max_deg_per_s2");
        }
        if (!readNodeIfPresent(motion, "gripper_max_units_per_s", cfg.motion.gripper_max_units_per_s)) {
            warnMissing(yamlPath, "robot.motion_limits.gripper_max_units_per_s");
        }
        if (!readNodeIfPresent(motion, "gripper_max_units_per_s2", cfg.motion.gripper_max_units_per_s2)) {
            warnMissing(yamlPath, "robot.motion_limits.gripper_max_units_per_s2");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.motion_limits' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    const cv::FileNode workspace = root["workspace_bounds_m"];
    if (!workspace.empty()) {
        if (!readNodeIfPresent(workspace, "x_min", cfg.workspace.x_min_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.x_min");
        }
        if (!readNodeIfPresent(workspace, "x_max", cfg.workspace.x_max_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.x_max");
        }
        if (!readNodeIfPresent(workspace, "y_min", cfg.workspace.y_min_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.y_min");
        }
        if (!readNodeIfPresent(workspace, "y_max", cfg.workspace.y_max_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.y_max");
        }
        if (!readNodeIfPresent(workspace, "z_min", cfg.workspace.z_min_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.z_min");
        }
        if (!readNodeIfPresent(workspace, "z_max", cfg.workspace.z_max_m)) {
            warnMissing(yamlPath, "robot.workspace_bounds_m.z_max");
        }
    } else {
        std::cerr << "loadRobotConfig: missing 'robot.workspace_bounds_m' in " << yamlPath
                  << " (using defaults)" << std::endl;
    }

    cfg.valid = true;
    return cfg;
}
