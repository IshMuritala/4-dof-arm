#ifndef INC_4DOF_ARM_SAFETY_CHECKS_H
#define INC_4DOF_ARM_SAFETY_CHECKS_H

#include <string>
#include <Eigen/Core>
#include "grasp_system/planning/fk.h"
#include "grasp_system/planning/robot_config.h"

struct SafetyResult {
    bool ok = true;
    std::string reason;
};


SafetyResult checkWorkspacePoint(const RobotConfig& cfg,
                                 const Eigen::Vector3f& point_m,
                                 const std::string& label);


SafetyResult checkJointLimits(const RobotConfig& cfg,
                              const JointAnglesDeg& joints,
                              const std::string& label);


SafetyResult checkGripperLimits(const RobotConfig& cfg,
                                int gripper_pos,
                                const std::string& label);


SafetyResult checkFkInWorkspace(const RobotConfig& cfg,
                                const JointAnglesDeg& joints,
                                const std::string& label);

#endif //INC_4DOF_ARM_SAFETY_CHECKS_H
