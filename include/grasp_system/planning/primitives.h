#ifndef INC_4DOF_ARM_PRIMITIVES_H
#define INC_4DOF_ARM_PRIMITIVES_H

#include <Eigen/Core>
#include <string>
#include <vector>
#include "grasp_system/planning/fk.h"
#include "grasp_system/planning/robot_config.h"

struct PrimitiveStep {
    std::string name;
    JointAnglesDeg joints;
    int gripper = 0;
};

struct PrimitivePlan {
    bool success = false;
    std::string reason;
    std::vector<PrimitiveStep> steps;
};

struct Pinch48dParams {
    float approach_offset_z_m = 0.06f;
    float lift_offset_z_m = 0.08f;
    float wrist_deg = 48.0f;
    bool prefer_elbow_down = true;
    bool enforce_workspace = true;
    int gripper_open = 1;
    int gripper_closed = 3700;

    bool gripper_dynamic = false;
    float gripper_margin_mm = 2.0f;
    float gripper_min_gap_mm = 6.0f;
    float gripper_max_gap_mm = 120.0f;
    float gripper_gap_open_mm = 120.0f;
    float gripper_gap_closed_mm = 0.0f;
    int gripper_pos_open = 1;
    int gripper_pos_closed = 3700;
};


Pinch48dParams loadPinch48dParams(const std::string& yamlPath);


PrimitivePlan buildPinch48dPlan(const RobotConfig& cfg,
                                const Pinch48dParams& params,
                                const Eigen::Vector3f& target_m,
                                float object_width_m = -1.0f);

#endif //INC_4DOF_ARM_PRIMITIVES_H
