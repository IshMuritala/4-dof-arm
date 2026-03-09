#ifndef INC_4DOF_ARM_IK_H
#define INC_4DOF_ARM_IK_H

#include <Eigen/Core>
#include <string>
#include "grasp_system/planning/fk.h"
#include "grasp_system/planning/robot_config.h"

struct IkResult {
    bool success = false;
    JointAnglesDeg angles;
    std::string reason;
};


IkResult solveIk(const RobotConfig& cfg,
                 const Eigen::Vector3f& target_m,
                 float desired_wrist_deg = 48.0f,
                 // prefer_elbow_down: choose the IK solution where elbow servo >= 90 deg
                 bool prefer_elbow_down = true);

#endif //INC_4DOF_ARM_IK_H
