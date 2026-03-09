#include "grasp_system/planning/primitives.h"

#include <opencv2/core.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include "grasp_system/planning/ik.h"
#include "grasp_system/planning/safety_checks.h"

namespace {

template <typename T>
void readNodeIfPresent(const cv::FileNode& node, const char* key, T& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> value;
    }
}

void readNodeIfPresent(const cv::FileNode& node, const char* key, bool& value) {
    const cv::FileNode child = node[key];
    if (child.empty()) {
        return;
    }
    if (child.isString()) {
        std::string s;
        child >> s;
        std::string lower;
        lower.resize(s.size());
        std::transform(s.begin(), s.end(), lower.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lower == "true" || lower == "yes" || lower == "on" || lower == "1") {
            value = true;
            return;
        }
        if (lower == "false" || lower == "no" || lower == "off" || lower == "0") {
            value = false;
            return;
        }
    }
    int tmp = 0;
    child >> tmp;
    value = (tmp != 0);
}

float clampFloat(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

int clampInt(int v, int min_v, int max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

int computeGripperClosePosition(const Pinch48dParams& params, float object_width_m) {
    if (!params.gripper_dynamic || object_width_m <= 0.0f) {
        return params.gripper_closed;
    }

    const float object_mm = object_width_m * 1000.0f;
    float target_gap_mm = object_mm + params.gripper_margin_mm;
    target_gap_mm = clampFloat(target_gap_mm, params.gripper_min_gap_mm, params.gripper_max_gap_mm);

    const float gap_open = params.gripper_gap_open_mm;
    const float gap_closed = params.gripper_gap_closed_mm;
    const float denom = (gap_closed - gap_open);
    if (std::abs(denom) < 1e-6f) {
        return params.gripper_closed;
    }

    float t = (target_gap_mm - gap_open) / denom;
    float pos = static_cast<float>(params.gripper_pos_open) +
                t * (static_cast<float>(params.gripper_pos_closed) -
                     static_cast<float>(params.gripper_pos_open));

    int minPos = std::min(params.gripper_pos_open, params.gripper_pos_closed);
    int maxPos = std::max(params.gripper_pos_open, params.gripper_pos_closed);
    return clampInt(static_cast<int>(std::lround(pos)), minPos, maxPos);
}

} // namespace

Pinch48dParams loadPinch48dParams(const std::string& yamlPath) {
    Pinch48dParams params;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "loadPinch48dParams: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return params;
    }
    if (!fs.isOpened()) {
        return params;
    }

    const cv::FileNode root = fs["primitives"];
    if (root.empty()) {
        return params;
    }
    const cv::FileNode pinch = root["pinch_48d"];
    if (pinch.empty()) {
        return params;
    }

    readNodeIfPresent(pinch, "approach_offset_z_m", params.approach_offset_z_m);
    readNodeIfPresent(pinch, "lift_offset_z_m", params.lift_offset_z_m);
    readNodeIfPresent(pinch, "wrist_deg", params.wrist_deg);
    readNodeIfPresent(pinch, "prefer_elbow_down", params.prefer_elbow_down);
    readNodeIfPresent(pinch, "enforce_workspace", params.enforce_workspace);
    readNodeIfPresent(pinch, "gripper_open", params.gripper_open);
    readNodeIfPresent(pinch, "gripper_closed", params.gripper_closed);
    readNodeIfPresent(pinch, "gripper_dynamic", params.gripper_dynamic);
    readNodeIfPresent(pinch, "gripper_margin_mm", params.gripper_margin_mm);
    readNodeIfPresent(pinch, "gripper_min_gap_mm", params.gripper_min_gap_mm);
    readNodeIfPresent(pinch, "gripper_max_gap_mm", params.gripper_max_gap_mm);
    readNodeIfPresent(pinch, "gripper_gap_open_mm", params.gripper_gap_open_mm);
    readNodeIfPresent(pinch, "gripper_gap_closed_mm", params.gripper_gap_closed_mm);
    readNodeIfPresent(pinch, "gripper_pos_open", params.gripper_pos_open);
    readNodeIfPresent(pinch, "gripper_pos_closed", params.gripper_pos_closed);

    return params;
}

PrimitivePlan buildPinch48dPlan(const RobotConfig& cfg,
                                const Pinch48dParams& params,
                                const Eigen::Vector3f& target_m,
                                float object_width_m) {
    PrimitivePlan plan;
    if (!cfg.valid) {
        plan.reason = "robot config not loaded";
        return plan;
    }

    Eigen::Vector3f approach = target_m;
    approach.z() += params.approach_offset_z_m;

    Eigen::Vector3f lift = target_m;
    lift.z() += params.lift_offset_z_m;

    if (params.enforce_workspace) {
        SafetyResult ws = checkWorkspacePoint(cfg, target_m, "target");
        if (!ws.ok) {
            plan.reason = ws.reason;
            return plan;
        }
        ws = checkWorkspacePoint(cfg, approach, "approach");
        if (!ws.ok) {
            plan.reason = ws.reason;
            return plan;
        }
        ws = checkWorkspacePoint(cfg, lift, "lift");
        if (!ws.ok) {
            plan.reason = ws.reason;
            return plan;
        }
    }

    IkResult ik_approach = solveIk(cfg, approach, params.wrist_deg, params.prefer_elbow_down);
    if (!ik_approach.success) {
        plan.reason = "IK failed for approach: " + ik_approach.reason;
        return plan;
    }
    SafetyResult check = checkJointLimits(cfg, ik_approach.angles, "approach");
    if (!check.ok) {
        plan.reason = check.reason;
        return plan;
    }
    if (params.enforce_workspace) {
        check = checkFkInWorkspace(cfg, ik_approach.angles, "approach");
        if (!check.ok) {
            plan.reason = check.reason;
            return plan;
        }
    }

    IkResult ik_grasp = solveIk(cfg, target_m, params.wrist_deg, params.prefer_elbow_down);
    if (!ik_grasp.success) {
        plan.reason = "IK failed for grasp: " + ik_grasp.reason;
        return plan;
    }
    check = checkJointLimits(cfg, ik_grasp.angles, "grasp");
    if (!check.ok) {
        plan.reason = check.reason;
        return plan;
    }
    if (params.enforce_workspace) {
        check = checkFkInWorkspace(cfg, ik_grasp.angles, "grasp");
        if (!check.ok) {
            plan.reason = check.reason;
            return plan;
        }
    }

    IkResult ik_lift = solveIk(cfg, lift, params.wrist_deg, params.prefer_elbow_down);
    if (!ik_lift.success) {
        plan.reason = "IK failed for lift: " + ik_lift.reason;
        return plan;
    }
    check = checkJointLimits(cfg, ik_lift.angles, "lift");
    if (!check.ok) {
        plan.reason = check.reason;
        return plan;
    }
    if (params.enforce_workspace) {
        check = checkFkInWorkspace(cfg, ik_lift.angles, "lift");
        if (!check.ok) {
            plan.reason = check.reason;
            return plan;
        }
    }

    const int gripper_close = computeGripperClosePosition(params, object_width_m);
    check = checkGripperLimits(cfg, params.gripper_open, "gripper_open");
    if (!check.ok) {
        plan.reason = check.reason;
        return plan;
    }
    check = checkGripperLimits(cfg, gripper_close, "gripper_close");
    if (!check.ok) {
        plan.reason = check.reason;
        return plan;
    }
    plan.steps.push_back({"approach_open", ik_approach.angles, params.gripper_open});
    plan.steps.push_back({"descend_open", ik_grasp.angles, params.gripper_open});
    plan.steps.push_back({"close", ik_grasp.angles, gripper_close});
    plan.steps.push_back({"lift", ik_lift.angles, gripper_close});
    plan.success = true;
    return plan;
}
