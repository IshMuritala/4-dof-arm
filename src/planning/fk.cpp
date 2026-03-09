#include "grasp_system/planning/fk.h"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;

constexpr float degToRad(float deg) {
    return deg * (kPi / 180.0f);
}

} // namespace

Eigen::Vector3f forwardKinematics(const RobotConfig& cfg, const JointAnglesDeg& angles) {
    // Coordinate frame:
    // +X forward, +Y left, +Z up. Base yaw rotates about +Z.
    // Mapping from servo angles to math angles (deg):
    // - base: 90 deg is forward => yaw_deg = base - 90
    // - shoulder: 0 deg forward, 90 deg up => pitch_deg = shoulder
    // - elbow: 0 deg straight (same direction as shoulder link), +90 bends forward
    //         => relative pitch = -elbow
    // - wrist: 90 deg forward, 0 deg down, 180 deg up => relative pitch = wrist - 90
    const float base_deg = angles.base + cfg.joint_offsets.base;
    const float shoulder_deg = angles.shoulder + cfg.joint_offsets.shoulder;
    const float elbow_deg = angles.elbow + cfg.joint_offsets.elbow;
    const float wrist_deg = angles.wrist + cfg.joint_offsets.wrist;

    const float yaw_rad = degToRad(base_deg - 90.0f);
    const float shoulder_rad = degToRad(shoulder_deg);
    const float elbow_rad = degToRad(-elbow_deg);
    const float wrist_rad = degToRad(wrist_deg - 90.0f);

    const float t1 = shoulder_rad;
    const float t2 = shoulder_rad + elbow_rad;
    const float t3 = shoulder_rad + elbow_rad + wrist_rad;

    const float L1 = cfg.geometry.shoulder_to_elbow_m;
    const float L2 = cfg.geometry.elbow_to_wrist_m;
    const float Lf = cfg.geometry.wrist_to_gripper_forward_m;
    const float Lu = cfg.geometry.wrist_to_gripper_up_m;

    const float r = L1 * std::cos(t1) + L2 * std::cos(t2) +
                    Lf * std::cos(t3) - Lu * std::sin(t3);
    const float z = cfg.geometry.base_height_m +
                    L1 * std::sin(t1) + L2 * std::sin(t2) +
                    Lf * std::sin(t3) + Lu * std::cos(t3);

    const float x = r * std::cos(yaw_rad);
    const float y = r * std::sin(yaw_rad);

    return Eigen::Vector3f(x, y, z);
}
