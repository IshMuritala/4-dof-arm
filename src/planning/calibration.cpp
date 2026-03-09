#include "grasp_system/planning/calibration.h"

#include <opencv2/core.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <vector>

namespace {

bool readNodeIfPresent(const cv::FileNode& node, const char* key, float& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> value;
        return true;
    }
    return false;
}

bool readNodeIfPresent(const cv::FileNode& node, const char* key, int& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> value;
        return true;
    }
    return false;
}

bool readMatrix4f(const cv::FileNode& node, Eigen::Matrix4f& out) {
    if (node.empty()) {
        return false;
    }
    out = Eigen::Matrix4f::Identity();

    if (node.type() == cv::FileNode::SEQ) {
        if (node.size() != 4) {
            return false;
        }
        for (int r = 0; r < 4; ++r) {
            const cv::FileNode row = node[r];
            if (row.empty() || row.type() != cv::FileNode::SEQ || row.size() != 4) {
                return false;
            }
            for (int c = 0; c < 4; ++c) {
                float v = 0.0f;
                row[c] >> v;
                out(r, c) = v;
            }
        }
        return true;
    }

    cv::Mat Tcv;
    node >> Tcv;
    if (Tcv.empty() || Tcv.rows != 4 || Tcv.cols != 4) {
        return false;
    }
    cv::Mat T32;
    Tcv.convertTo(T32, CV_32F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out(r, c) = T32.at<float>(r, c);
        }
    }
    return true;
}

// Fallback parser for plain YAML list-of-lists:
// calibration:
//   T_robot_from_cam:
//     - [a, b, c, d]
//     - [e, f, g, h]
//     - [i, j, k, l]
//     - [0, 0, 0, 1]
bool extractNumbers(const std::string& line, std::vector<float>& out) {
    static const std::regex kFloatRe(R"([-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?)");
    for (std::sregex_iterator it(line.begin(), line.end(), kFloatRe), end; it != end; ++it) {
        try {
            out.push_back(std::stof(it->str()));
        } catch (...) {
            return false;
        }
    }
    return true;
}

bool parsePlainYamlCalibration(const std::string& yamlPath, Calibration& calib) {
    std::ifstream in(yamlPath);
    if (!in.is_open()) {
        return false;
    }

    std::vector<float> vals;
    bool inMatrix = false;
    bool haveRms = false;
    bool haveNum = false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.find("T_robot_from_cam") != std::string::npos) {
            inMatrix = true;
            continue;
        }
        if (inMatrix) {
            if (line.find('[') != std::string::npos) {
                if (!extractNumbers(line, vals)) {
                    return false;
                }
                if (vals.size() >= 16) {
                    inMatrix = false;
                }
                continue;
            } else if (line.find("rms_error_m") != std::string::npos ||
                       line.find("num_points") != std::string::npos) {
                inMatrix = false;
            }
        }
        if (line.find("rms_error_m") != std::string::npos) {
            std::vector<float> nums;
            if (extractNumbers(line, nums) && !nums.empty()) {
                calib.rms_error_m = nums[0];
                haveRms = true;
            }
        } else if (line.find("num_points") != std::string::npos) {
            std::vector<float> nums;
            if (extractNumbers(line, nums) && !nums.empty()) {
                calib.num_points = static_cast<int>(nums[0]);
                haveNum = true;
            }
        }
    }

    if (vals.size() < 16) {
        return false;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 16; ++i) {
        T(i / 4, i % 4) = vals[static_cast<std::size_t>(i)];
    }

    calib.T = T;
    calib.R = T.block<3, 3>(0, 0);
    calib.t = T.block<3, 1>(0, 3);
    calib.valid = true;

    if (!haveRms) {
        calib.rms_error_m = 0.0f;
    }
    if (!haveNum) {
        calib.num_points = 0;
    }

    return true;
}

} // namespace

Calibration loadCalibration(const std::string& yamlPath) {
    Calibration calib;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        if (parsePlainYamlCalibration(yamlPath, calib)) {
            return calib;
        }
        std::cerr << "loadCalibration: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return calib;
    }
    if (!fs.isOpened()) {
        if (parsePlainYamlCalibration(yamlPath, calib)) {
            return calib;
        }
        std::cerr << "loadCalibration: could not open " << yamlPath << std::endl;
        return calib;
    }

    const cv::FileNode root = fs["calibration"];
    if (root.empty()) {
        if (parsePlainYamlCalibration(yamlPath, calib)) {
            return calib;
        }
        std::cerr << "loadCalibration: missing 'calibration' root in " << yamlPath << std::endl;
        return calib;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    if (!readMatrix4f(root["T_robot_from_cam"], T)) {
        if (parsePlainYamlCalibration(yamlPath, calib)) {
            return calib;
        }
        std::cerr << "loadCalibration: missing or invalid T_robot_from_cam in " << yamlPath << std::endl;
        return calib;
    }

    calib.T = T;
    calib.R = T.block<3, 3>(0, 0);
    calib.t = T.block<3, 1>(0, 3);

    readNodeIfPresent(root, "rms_error_m", calib.rms_error_m);
    readNodeIfPresent(root, "num_points", calib.num_points);

    calib.valid = true;
    return calib;
}

Eigen::Vector3f transformPoint(const Calibration& calib, const Eigen::Vector3f& camPoint) {
    return calib.R * camPoint + calib.t;
}
