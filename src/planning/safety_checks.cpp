#include "grasp_system/planning/safety_checks.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {

std::string vecToString(const Eigen::Vector3f& v) {
    std::ostringstream oss;
    oss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
    return oss.str();
}

bool within(float value, float min_v, float max_v, float eps = 1e-4f) {
    return value >= (min_v - eps) && value <= (max_v + eps);
}

SafetyResult fail(const std::string& reason) {
    SafetyResult r;
    r.ok = false;
    r.reason = reason;
    return r;
}

} // namespace

SafetyResult checkWorkspacePoint(const RobotConfig& cfg,
                                 const Eigen::Vector3f& point_m,
                                 const std::string& label) {
    if (!cfg.valid) {
        return fail(label + ": robot config not loaded");
    }

    const WorkspaceBounds& ws = cfg.workspace;
    if (!within(point_m.x(), ws.x_min_m, ws.x_max_m) ||
        !within(point_m.y(), ws.y_min_m, ws.y_max_m) ||
        !within(point_m.z(), ws.z_min_m, ws.z_max_m)) {
        std::ostringstream oss;
        oss << label << ": outside workspace bounds. point=" << vecToString(point_m)
            << " bounds[x:" << ws.x_min_m << "," << ws.x_max_m
            << " y:" << ws.y_min_m << "," << ws.y_max_m
            << " z:" << ws.z_min_m << "," << ws.z_max_m << "]";
        return fail(oss.str());
    }

    return {};
}

SafetyResult checkJointLimits(const RobotConfig& cfg,
                              const JointAnglesDeg& joints,
                              const std::string& label) {
    if (!cfg.valid) {
        return fail(label + ": robot config not loaded");
    }

    const JointLimitsDeg& lim = cfg.joint_limits;
    if (!within(joints.base, lim.base_min, lim.base_max)) {
        return fail(label + ": base angle out of limits");
    }
    if (!within(joints.shoulder, lim.shoulder_min, lim.shoulder_max)) {
        return fail(label + ": shoulder angle out of limits");
    }
    if (!within(joints.elbow, lim.elbow_min, lim.elbow_max)) {
        return fail(label + ": elbow angle out of limits");
    }
    if (!within(joints.wrist, lim.wrist_min, lim.wrist_max)) {
        return fail(label + ": wrist angle out of limits");
    }

    return {};
}

SafetyResult checkGripperLimits(const RobotConfig& cfg,
                                int gripper_pos,
                                const std::string& label) {
    if (!cfg.valid) {
        return fail(label + ": robot config not loaded");
    }

    const int min_g = std::min(cfg.gripper.open, cfg.gripper.closed);
    const int max_g = std::max(cfg.gripper.open, cfg.gripper.closed);
    if (gripper_pos < min_g || gripper_pos > max_g) {
        std::ostringstream oss;
        oss << label << ": gripper position out of limits (" << gripper_pos
            << " not in [" << min_g << "," << max_g << "])";
        return fail(oss.str());
    }

    return {};
}

SafetyResult checkFkInWorkspace(const RobotConfig& cfg,
                                const JointAnglesDeg& joints,
                                const std::string& label) {
    if (!cfg.valid) {
        return fail(label + ": robot config not loaded");
    }
    const Eigen::Vector3f p = forwardKinematics(cfg, joints);
    return checkWorkspacePoint(cfg, p, label + " (FK)");
}
