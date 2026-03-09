#ifndef INC_4DOF_ARM_FK_H
#define INC_4DOF_ARM_FK_H

#include <Eigen/Core>
#include "grasp_system/planning/robot_config.h"

struct JointAnglesDeg {
    float base = 90.0f;
    float shoulder = 90.0f;
    float elbow = 90.0f;
    float wrist = 90.0f;
};

// Forward kinematics
Eigen::Vector3f forwardKinematics(const RobotConfig& cfg, const JointAnglesDeg& angles);

#endif //INC_4DOF_ARM_FK_H
