#ifndef INC_4DOF_ARM_ROBOT_CONFIG_H
#define INC_4DOF_ARM_ROBOT_CONFIG_H

#include <string>

struct RobotGeometry {
    float base_height_m = 0.094f;
    float shoulder_to_elbow_m = 0.105f;
    float elbow_to_wrist_m = 0.095f;
    float wrist_to_gripper_forward_m = 0.142f;
    float wrist_to_gripper_up_m = -0.01f;
};

struct JointLimitsDeg {
    float base_min = 0.0f;
    float base_max = 180.0f;
    float shoulder_min = 0.0f;
    float shoulder_max = 180.0f;
    float elbow_min = 0.0f;
    float elbow_max = 150.0f;
    float wrist_min = 48.0f;
    float wrist_max = 115.0f;
};

struct JointOffsetsDeg {
    float base = 0.0f;
    float shoulder = -3.0f;
    float elbow = -3.0f;
    float wrist = 10.0f;
};

struct HomePoseDeg {
    float base = 90.0f;
    float shoulder = 90.0f;
    float elbow = 90.0f;
    float wrist = 90.0f;
};

struct GripperLimits {
    int open = 1000;
    int closed = 4000;
    int home = 2800;
};

struct MotionLimits {
    float joint_max_deg_per_s = 0.0f;
    float joint_max_deg_per_s2 = 0.0f;
    float gripper_max_units_per_s = 0.0f;
    float gripper_max_units_per_s2 = 0.0f;
};

struct WorkspaceBounds {
    float x_min_m = 0.12f;
    float x_max_m = 0.30f;
    float y_min_m = -0.14f;
    float y_max_m = 0.14f;
    float z_min_m = 0.015f;
    float z_max_m = 0.30f;
};

struct RobotConfig {
    RobotGeometry geometry;
    JointLimitsDeg joint_limits;
    JointOffsetsDeg joint_offsets;
    HomePoseDeg home;
    GripperLimits gripper;
    MotionLimits motion;
    WorkspaceBounds workspace;
    bool valid = false;
};

RobotConfig loadRobotConfig(const std::string& yamlPath);

#endif //INC_4DOF_ARM_ROBOT_CONFIG_H
