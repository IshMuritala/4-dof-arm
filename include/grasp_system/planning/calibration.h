#ifndef INC_4DOF_ARM_CALIBRATION_H
#define INC_4DOF_ARM_CALIBRATION_H

#include <Eigen/Core>
#include <string>

struct Calibration {
    bool valid = false;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float rms_error_m = 0.0f;
    int num_points = 0;
};


Calibration loadCalibration(const std::string& yamlPath);


Eigen::Vector3f transformPoint(const Calibration& calib, const Eigen::Vector3f& camPoint);

#endif //INC_4DOF_ARM_CALIBRATION_H
