#include "grasp_system/planning/ik.h"
#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;

constexpr float degToRad(float deg) {
    return deg * (kPi / 180.0f);
}

constexpr float radToDeg(float rad) {
    return rad * (180.0f / kPi);
}

bool within(float value, float min_v, float max_v, float eps = 1e-3f) {
    return value >= (min_v - eps) && value <= (max_v + eps);
}

struct IkCandidate {
    bool valid = false;
    JointAnglesDeg angles;
};

} // namespace

IkResult solveIk(const RobotConfig& cfg,
                 const Eigen::Vector3f& target_m,
                 float desired_wrist_deg,
                 bool prefer_elbow_down) {
    IkResult result;

    if (!cfg.valid) {
        result.reason = "robot config not loaded";
        return result;
    }

    if (!within(desired_wrist_deg, cfg.joint_limits.wrist_min, cfg.joint_limits.wrist_max)) {
        result.reason = "desired wrist angle out of limits";
        return result;
    }

    const float x = target_m.x();
    const float y = target_m.y();
    const float z = target_m.z();


    const float yaw_rad = std::atan2(y, x);
    const float base_deg = radToDeg(yaw_rad) + 90.0f - cfg.joint_offsets.base;

    if (!within(base_deg, cfg.joint_limits.base_min, cfg.joint_limits.base_max)) {
        result.reason = "base angle out of limits";
        return result;
    }

    const float r = std::sqrt(x * x + y * y);
    const float x_p = r;
    const float z_p = z - cfg.geometry.base_height_m;

    const float L1 = cfg.geometry.shoulder_to_elbow_m;
    const float L2 = cfg.geometry.elbow_to_wrist_m;
    const float Lf = cfg.geometry.wrist_to_gripper_forward_m;
    const float Lu = cfg.geometry.wrist_to_gripper_up_m;

    // Wrist mapping: 90 deg = forward (aligned with forearm),
    // 0 deg = down, 180 deg = up -> relative pitch = wrist - 90.
    const float desired_wrist_eff = desired_wrist_deg + cfg.joint_offsets.wrist;
    const float wrist_rel_rad = degToRad(desired_wrist_eff - 90.0f);

    const float v_fwd = Lf * std::cos(wrist_rel_rad) - Lu * std::sin(wrist_rel_rad);
    const float v_up = Lf * std::sin(wrist_rel_rad) + Lu * std::cos(wrist_rel_rad);
    const float a = L2 + v_fwd;
    const float b = v_up;
    const float L2_eff = std::sqrt(a * a + b * b);
    const float delta = std::atan2(b, a);

    const float dist2 = x_p * x_p + z_p * z_p;
    const float denom = 2.0f * L1 * L2_eff;
    if (denom <= 1e-6f) {
        result.reason = "invalid link lengths";
        return result;
    }

    float cos_theta2p = (dist2 - (L1 * L1 + L2_eff * L2_eff)) / denom;
    if (cos_theta2p < -1.0f || cos_theta2p > 1.0f) {
        result.reason = "target unreachable";
        return result;
    }
    if (cos_theta2p < -1.0f) cos_theta2p = -1.0f;
    if (cos_theta2p > 1.0f) cos_theta2p = 1.0f;

    const float sin_theta2p = std::sqrt(std::max(0.0f, 1.0f - cos_theta2p * cos_theta2p));

    const float theta2p_a = std::atan2(sin_theta2p, cos_theta2p);
    const float theta2p_b = std::atan2(-sin_theta2p, cos_theta2p);

    auto buildCandidate = [&](float theta2p) -> IkCandidate {
        IkCandidate cand;
        const float theta1 = std::atan2(z_p, x_p) -
                             std::atan2(L2_eff * std::sin(theta2p),
                                        L1 + L2_eff * std::cos(theta2p));

        // Elbow mapping: elbow servo 0 deg = straight (same direction as shoulder link),
        // 90 deg = forward, 180 deg = down -> math relative angle = -elbow.
        // "Elbow-down" = elbow servo angle >= 90 deg
        // "Elbow-up" = elbow servo angle < 90 deg
        const float theta2 = theta2p - delta;

        const float shoulder_deg = radToDeg(theta1) - cfg.joint_offsets.shoulder;
        const float elbow_deg = -radToDeg(theta2) - cfg.joint_offsets.elbow;
        const float wrist_deg = desired_wrist_deg;

        if (!within(shoulder_deg, cfg.joint_limits.shoulder_min, cfg.joint_limits.shoulder_max)) {
            return cand;
        }
        if (!within(elbow_deg, cfg.joint_limits.elbow_min, cfg.joint_limits.elbow_max)) {
            return cand;
        }
        if (!within(wrist_deg, cfg.joint_limits.wrist_min, cfg.joint_limits.wrist_max)) {
            return cand;
        }

        cand.valid = true;
        cand.angles.base = base_deg;
        cand.angles.shoulder = shoulder_deg;
        cand.angles.elbow = elbow_deg;
        cand.angles.wrist = wrist_deg;
        return cand;
    };

    IkCandidate cand_a = buildCandidate(theta2p_a);
    IkCandidate cand_b = buildCandidate(theta2p_b);

    if (!cand_a.valid && !cand_b.valid) {
        result.reason = "no IK solution within joint limits";
        return result;
    }

    auto isElbowDown = [&](const IkCandidate& cand) {
        return cand.angles.elbow >= 90.0f;
    };

    if (cand_a.valid && cand_b.valid) {
        if (prefer_elbow_down) {
            result.angles = isElbowDown(cand_a) ? cand_a.angles : cand_b.angles;
        } else {
            result.angles = isElbowDown(cand_a) ? cand_b.angles : cand_a.angles;
        }
    } else {
        result.angles = cand_a.valid ? cand_a.angles : cand_b.angles;
    }

    result.success = true;
    return result;
}
