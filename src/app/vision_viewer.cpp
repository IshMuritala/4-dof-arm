#include "grasp_system/app/vision_viewer.h"

#include "grasp_system/camera/orbbec_camera.h"
#include "grasp_system/perception/pointcloud_builder.h"
#include "grasp_system/perception/segmentation.h"
#include "grasp_system/perception/features_color.h"
#include "grasp_system/perception/scene_summary.h"
#include "grasp_system/perception/tracking.h"
#include "grasp_system/planning/calibration.h"
#include "grasp_system/planning/primitives.h"
#include "grasp_system/planning/robot_config.h"
#include "grasp_system/planning/safety_checks.h"
#include "grasp_system/comms/serial_arduino.h"
#include "grasp_system/viz/pcl_viewer.h"

#include <opencv2/core.hpp>
#include <pcl/common/point_tests.h>
#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <deque>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>
#include <optional>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <array>
#include <thread>
#include <vector>

namespace {

enum class DisplayMode {
    Raw,
    Objects,
    Plane,
    Clusters
};

static const char* modeLabel(DisplayMode mode) {
    switch (mode) {
        case DisplayMode::Raw:
            return "Mode: Raw";
        case DisplayMode::Objects:
            return "Mode: Objects-only (default)";
        case DisplayMode::Plane:
            return "Mode: Plane-only";
        case DisplayMode::Clusters:
            return "Mode: Clusters";
        default:
            return "Mode: Unknown";
    }
}

static DisplayMode nextMode(DisplayMode mode) {
    switch (mode) {
        case DisplayMode::Objects:
            return DisplayMode::Raw;
        case DisplayMode::Raw:
            return DisplayMode::Plane;
        case DisplayMode::Plane:
            return DisplayMode::Clusters;
        case DisplayMode::Clusters:
        default:
            return DisplayMode::Objects;
    }
}

static CameraIntrinsics toSceneIntrinsics(const OrbbecCamera::Intrinsics& intr) {
    CameraIntrinsics out;
    out.fx = intr.fx;
    out.fy = intr.fy;
    out.cx = intr.cx;
    out.cy = intr.cy;
    out.width = intr.width;
    out.height = intr.height;
    return out;
}

constexpr int kAckTimeoutMs = 1000;
constexpr int kDoneTimeoutMs = 12000;
constexpr int kReadyBase = 90;
constexpr int kReadyShoulder = 145;
constexpr int kReadyElbow = 145;
constexpr int kReadyWrist = 80;

static bool readEepromAngles(SerialArduino& arduino, JointAnglesDeg& joints, int& gripper) {
    uint8_t responseType = 0;
    uint8_t responseData[MAX_MESSAGE_LENGTH] = {0};
    uint8_t responseLen = 0;

    if (!arduino.sendGetAngles()) {
        return false;
    }
    if (!arduino.waitForResponse(responseType, responseData, responseLen, 1000)) {
        return false;
    }
    if (responseType != RESP_ACK || responseLen != sizeof(GetAnglesData)) {
        return false;
    }
    const GetAnglesData* angles = reinterpret_cast<const GetAnglesData*>(responseData);
    joints.base = static_cast<float>(angles->base);
    joints.shoulder = static_cast<float>(angles->shoulder);
    joints.elbow = static_cast<float>(angles->elbow);
    joints.wrist = static_cast<float>(angles->wrist);
    gripper = static_cast<int>(angles->gripper);
    return true;
}

static bool waitForArduinoReady(SerialArduino& arduino, int totalTimeoutMs) {
    using namespace std::chrono;
    const auto start = steady_clock::now();
    while (duration_cast<milliseconds>(steady_clock::now() - start).count() < totalTimeoutMs) {
        if (!arduino.sendPing()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        uint8_t responseType = 0;
        uint8_t responseData[MAX_MESSAGE_LENGTH] = {0};
        uint8_t responseLen = 0;
        if (arduino.waitForResponse(responseType, responseData, responseLen, 500)) {
            if (responseType == RESP_ACK) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

static bool sendMoveAndWait(SerialArduino& arduino,
                            const RobotConfig& cfg,
                            const JointAnglesDeg& joints,
                            int gripper,
                            const std::string& label,
                            bool enforce_workspace) {
    SafetyResult check = checkJointLimits(cfg, joints, label);
    if (!check.ok) {
        std::cerr << "Safety check failed: " << check.reason << std::endl;
        return false;
    }
    check = checkGripperLimits(cfg, gripper, label);
    if (!check.ok) {
        std::cerr << "Safety check failed: " << check.reason << std::endl;
        return false;
    }
    if (enforce_workspace) {
        check = checkFkInWorkspace(cfg, joints, label);
        if (!check.ok) {
            std::cerr << "Safety check failed: " << check.reason << std::endl;
            return false;
        }
    }

    const uint16_t base = static_cast<uint16_t>(std::lround(joints.base));
    const uint16_t shoulder = static_cast<uint16_t>(std::lround(joints.shoulder));
    const uint16_t elbow = static_cast<uint16_t>(std::lround(joints.elbow));
    const uint16_t wrist = static_cast<uint16_t>(std::lround(joints.wrist));
    const uint16_t gripper_u = static_cast<uint16_t>(gripper);

    std::cout << "Step: " << label << std::endl;
    if (!arduino.sendMoveJoints(base, shoulder, elbow, wrist, gripper_u)) {
        std::cerr << "Failed to send move command for step: " << label << std::endl;
        return false;
    }
    if (!arduino.waitForAckAndDone(kAckTimeoutMs, kDoneTimeoutMs)) {
        std::cerr << "Timeout waiting for ACK/DONE on step: " << label << std::endl;
        return false;
    }
    return true;
}

static std::string joinPath(const std::string& dir, const std::string& file) {
    if (dir.empty()) {
        return file;
    }
    if (dir.back() == '/' || dir.back() == '\\') {
        return dir + file;
    }
    return dir + "/" + file;
}

static int64_t timestampMs() {
    using namespace std::chrono;
    auto now = system_clock::now();
    return duration_cast<milliseconds>(now.time_since_epoch()).count();
}

static std::string timestampMsString() {
    return std::to_string(timestampMs());
}

static std::string formatRobotCentroids(const SceneSummary& summary,
                                        const Calibration* calib,
                                        int precision = 3) {
    if (!summary.valid || calib == nullptr || !calib->valid) {
        return {};
    }
    std::ostringstream out;
    out << "Robot-frame centroids:\n";
    out.setf(std::ios::fixed);
    out << std::setprecision(precision);
    for (const auto& obj : summary.objects) {
        Eigen::Vector3f rc = transformPoint(*calib, obj.features.centroid);
        out << "  " << obj.id << ": ("
            << rc.x() << ", " << rc.y() << ", " << rc.z() << ")\n";
    }
    return out.str();
}

static Eigen::Vector3f getCentroidForTrack(const TrackedObject& obj) {
    if (obj.has_smoothed) {
        return obj.centroid_smoothed;
    }
    return obj.features.centroid;
}

static Eigen::Vector3f getBboxSizeForTrack(const TrackedObject& obj) {
    if (obj.has_smoothed) {
        return obj.bbox_max_smoothed - obj.bbox_min_smoothed;
    }
    return obj.features.bbox_size;
}

static bool ensureParentDir(const std::string& path) {
    if (path.empty()) {
        return false;
    }
    std::filesystem::path p(path);
    auto parent = p.parent_path();
    if (parent.empty()) {
        return true;
    }
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    return !ec;
}

static std::string escapeJson(const std::string& s) {
    std::ostringstream oss;
    for (char c : s) {
        switch (c) {
            case '\\': oss << "\\\\"; break;
            case '\"': oss << "\\\""; break;
            case '\b': oss << "\\b"; break;
            case '\f': oss << "\\f"; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    oss << "\\u"
                        << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(c))
                        << std::dec << std::setfill(' ');
                } else {
                    oss << c;
                }
        }
    }
    return oss.str();
}

static std::string shellEscape(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') {
            out += "'\\''";
        } else {
            out += c;
        }
    }
    out += "'";
    return out;
}

static std::string trimWhitespace(const std::string& s) {
    std::size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    std::size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static bool jsonExtractBool(const std::string& json,
                            const std::string& key,
                            bool& out) {
    std::string needle = "\"" + key + "\"";
    std::size_t pos = json.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) {
        return false;
    }
    pos = json.find_first_not_of(" \t\r\n", pos + 1);
    if (pos == std::string::npos) {
        return false;
    }
    if (json.compare(pos, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (json.compare(pos, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

static bool jsonExtractStringOrNull(const std::string& json,
                                    const std::string& key,
                                    std::string& out,
                                    bool& isNull) {
    std::string needle = "\"" + key + "\"";
    std::size_t pos = json.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    pos = json.find(':', pos + needle.size());
    if (pos == std::string::npos) {
        return false;
    }
    pos = json.find_first_not_of(" \t\r\n", pos + 1);
    if (pos == std::string::npos) {
        return false;
    }
    if (json.compare(pos, 4, "null") == 0) {
        isNull = true;
        out.clear();
        return true;
    }
    if (json[pos] != '"') {
        return false;
    }
    isNull = false;
    pos++;
    std::string value;
    value.reserve(32);
    while (pos < json.size()) {
        char c = json[pos++];
        if (c == '\\' && pos < json.size()) {
            char esc = json[pos++];
            switch (esc) {
                case '"': value.push_back('"'); break;
                case '\\': value.push_back('\\'); break;
                case 'n': value.push_back('\n'); break;
                case 'r': value.push_back('\r'); break;
                case 't': value.push_back('\t'); break;
                case 'u': {
                    if (pos + 3 < json.size()) {
                        auto hexVal = [](char h) -> int {
                            if (h >= '0' && h <= '9') return h - '0';
                            if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
                            if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
                            return -1;
                        };
                        int code = 0;
                        bool ok = true;
                        for (int i = 0; i < 4; ++i) {
                            int v = hexVal(json[pos + i]);
                            if (v < 0) {
                                ok = false;
                                break;
                            }
                            code = (code << 4) | v;
                        }
                        if (ok) {
                            pos += 4;
                            if (code <= 0x7F) {
                                value.push_back(static_cast<char>(code));
                            } else if (code <= 0x7FF) {
                                value.push_back(static_cast<char>(0xC0 | ((code >> 6) & 0x1F)));
                                value.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                            } else {
                                value.push_back(static_cast<char>(0xE0 | ((code >> 12) & 0x0F)));
                                value.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
                                value.push_back(static_cast<char>(0x80 | (code & 0x3F)));
                            }
                            break;
                        }
                    }
                    value.push_back('u');
                    break;
                }
                default: value.push_back(esc); break;
            }
            continue;
        }
        if (c == '"') {
            break;
        }
        value.push_back(c);
    }
    out = value;
    return true;
}

static bool jsonExtractString(const std::string& json,
                              const std::string& key,
                              std::string& out) {
    bool isNull = false;
    if (!jsonExtractStringOrNull(json, key, out, isNull)) {
        return false;
    }
    if (isNull) {
        return false;
    }
    return !out.empty();
}

static std::string toLowerCopy(const std::string& s) {
    std::string out = s;
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

static std::string runCommandCapture(const std::string& cmd, int& exitCode) {
    exitCode = -1;
    std::array<char, 512> buffer{};
    std::string output;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return output;
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
        output.append(buffer.data());
    }
    int status = pclose(pipe);
    if (status != -1) {
        if (WIFEXITED(status)) {
            exitCode = WEXITSTATUS(status);
        } else {
            exitCode = status;
        }
    }
    return output;
}

static std::string formatModelRequestJson(const SceneSummary& summary,
                                          int64_t timestamp_ms,
                                          int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "{";
    oss << "\"timestamp_ms\":" << timestamp_ms << ",";
    oss << "\"objects\":[";
    for (std::size_t i = 0; i < summary.objects.size(); ++i) {
        const auto& obj = summary.objects[i];
        if (i > 0) {
            oss << ",";
        }
        const auto& f = obj.features;
        float width_m = std::max(f.bbox_size.x(), f.bbox_size.y());
        oss << "{";
        oss << "\"id\":\"" << escapeJson(obj.id) << "\",";
        oss << "\"width_m\":" << width_m;
        oss << "}";
    }
    oss << "]";
    oss << "}";
    return oss.str();
}

static bool appendJsonlLine(const std::string& path, const std::string& line) {
    if (path.empty()) {
        return false;
    }
    if (!ensureParentDir(path)) {
        return false;
    }
    std::ofstream out(path, std::ios::app);
    if (!out) {
        return false;
    }
    out << line << "\n";
    return true;
}

static bool writeFile(const std::string& path, const std::string& contents) {
    if (path.empty()) {
        return false;
    }
    if (!ensureParentDir(path)) {
        return false;
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        return false;
    }
    out << contents;
    return true;
}

static bool moveFile(const std::string& from, const std::string& to) {
    if (from.empty() || to.empty()) {
        return false;
    }
    if (!ensureParentDir(to)) {
        return false;
    }
    std::error_code ec;
    std::filesystem::rename(from, to, ec);
    if (!ec) {
        return true;
    }
    std::filesystem::copy_file(from, to, std::filesystem::copy_options::overwrite_existing, ec);
    if (ec) {
        return false;
    }
    std::filesystem::remove(from, ec);
    return true;
}

struct ModelDecision {
    bool valid = false;
    bool confirm_needed = false;
    bool target_is_null = true;
    std::string target_id;
    std::string reason;
    std::string raw_json;
};

static std::optional<ModelDecision> parseModelDecision(const std::string& json) {
    ModelDecision out;
    out.raw_json = json;
    bool ok_confirm = jsonExtractBool(json, "confirm_needed", out.confirm_needed);
    bool ok_target = jsonExtractStringOrNull(json, "target_object_id",
                                             out.target_id, out.target_is_null);
    bool reason_is_null = false;
    bool ok_reason = jsonExtractStringOrNull(json, "reason", out.reason, reason_is_null);
    if (!ok_confirm || !ok_target) {
        return std::nullopt;
    }
    if (!ok_reason) {
        out.reason.clear();
    }
    out.valid = true;
    return out;
}

struct IntentDecision {
    bool valid = false;
    std::string action;
    std::string question;
    std::string raw_json;
};

static std::optional<IntentDecision> parseIntentDecision(const std::string& json) {
    IntentDecision out;
    out.raw_json = json;
    std::string action;
    if (!jsonExtractString(json, "action", action)) {
        return std::nullopt;
    }
    action = toLowerCopy(action);
    if (action != "pick" && action != "place_back" && action != "clarify") {
        return std::nullopt;
    }
    bool question_is_null = false;
    std::string question;
    if (!jsonExtractStringOrNull(json, "clarification_question", question, question_is_null)) {
        return std::nullopt;
    }
    if (action == "clarify") {
        if (question_is_null || question.empty()) {
            return std::nullopt;
        }
    }
    out.action = action;
    out.question = question;
    out.valid = true;
    return out;
}

static float percentile(std::vector<float>& values, float p) {
    if (values.empty()) {
        return 0.0f;
    }
    if (p <= 0.0f) {
        return *std::min_element(values.begin(), values.end());
    }
    if (p >= 1.0f) {
        return *std::max_element(values.begin(), values.end());
    }
    std::sort(values.begin(), values.end());
    float idx = p * static_cast<float>(values.size() - 1);
    std::size_t lo = static_cast<std::size_t>(std::floor(idx));
    std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
    if (lo == hi) {
        return values[lo];
    }
    float t = idx - static_cast<float>(lo);
    return values[lo] * (1.0f - t) + values[hi] * t;
}

struct WidthEstimationParams {
    float width_percentile_lo = 0.15f;
    float width_percentile_hi = 0.85f;
    bool radius_use_percentile = true;
    float radius_percentile_hi = 0.95f;
    float radius_inflation = 1.10f;
    float radius_min_m = 0.03f;
    float radius_fixed_m = 0.05f;
};

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

WidthEstimationParams loadWidthEstimationParams(const std::string& yamlPath) {
    WidthEstimationParams params;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception&) {
        return params;
    }
    if (!fs.isOpened()) {
        return params;
    }
    const cv::FileNode node = fs["width_estimation"];
    if (node.empty()) {
        return params;
    }
    readNodeIfPresent(node, "width_percentile_lo", params.width_percentile_lo);
    readNodeIfPresent(node, "width_percentile_hi", params.width_percentile_hi);
    readNodeIfPresent(node, "radius_use_percentile", params.radius_use_percentile);
    readNodeIfPresent(node, "radius_percentile_hi", params.radius_percentile_hi);
    readNodeIfPresent(node, "radius_inflation", params.radius_inflation);
    readNodeIfPresent(node, "radius_min_m", params.radius_min_m);
    readNodeIfPresent(node, "radius_fixed_m", params.radius_fixed_m);

    if (params.width_percentile_hi < params.width_percentile_lo) {
        std::swap(params.width_percentile_hi, params.width_percentile_lo);
    }
    params.width_percentile_lo = std::max(0.0f, std::min(1.0f, params.width_percentile_lo));
    params.width_percentile_hi = std::max(0.0f, std::min(1.0f, params.width_percentile_hi));
    params.radius_percentile_hi = std::max(0.0f, std::min(1.0f, params.radius_percentile_hi));
    params.radius_inflation = std::max(0.0f, params.radius_inflation);
    params.radius_min_m = std::max(0.0f, params.radius_min_m);
    params.radius_fixed_m = std::max(0.0f, params.radius_fixed_m);
    return params;
}

// Rebuild a higher-resolution cluster for width estimation by gathering points
// from the full objects cloud near the downsampled cluster centroid/bounds
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildFullClusterForWidth(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cluster,
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr objects,
        const ClusterParams& params,
        const WidthEstimationParams& widthParams) {
    if (!cluster || cluster->empty() || !objects || objects->empty()) {
        return nullptr;
    }

    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    std::size_t count = 0;

    for (const auto& pt : cluster->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        minX = std::min(minX, pt.x);
        minY = std::min(minY, pt.y);
        minZ = std::min(minZ, pt.z);
        maxX = std::max(maxX, pt.x);
        maxY = std::max(maxY, pt.y);
        maxZ = std::max(maxZ, pt.z);
        centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
        count++;
    }
    if (count == 0) {
        return nullptr;
    }
    centroid /= static_cast<float>(count);

    float pad = std::max(params.tolerance_m, params.voxel_leaf_m);
    minX -= pad; minY -= pad; minZ -= pad;
    maxX += pad; maxY += pad; maxZ += pad;

    float radius = 0.0f;
    if (widthParams.radius_use_percentile) {
        std::vector<float> dists;
        dists.reserve(cluster->points.size());
        for (const auto& pt : cluster->points) {
            if (!pcl::isFinite(pt)) {
                continue;
            }
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            dists.push_back((p - centroid).norm());
        }
        if (!dists.empty()) {
            float p = percentile(dists, widthParams.radius_percentile_hi);
            radius = p * widthParams.radius_inflation;
        }
    }
    if (radius <= 0.0f) {
        radius = widthParams.radius_fixed_m;
    }
    float minRadius = std::max(widthParams.radius_min_m, params.tolerance_m * 2.0f);
    if (radius < minRadius) {
        radius = minRadius;
    }
    float radius2 = radius * radius;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full(
            new pcl::PointCloud<pcl::PointXYZRGB>());
    full->points.reserve(cluster->points.size() * 4);
    for (const auto& pt : objects->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        if (pt.x < minX || pt.x > maxX) continue;
        if (pt.y < minY || pt.y > maxY) continue;
        if (pt.z < minZ || pt.z > maxZ) continue;
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        if ((p - centroid).squaredNorm() > radius2) {
            continue;
        }
        full->points.push_back(pt);
    }
    full->width = static_cast<uint32_t>(full->points.size());
    full->height = 1;
    full->is_dense = true;
    return full;
}

// Robust width from cluster points using percentile bounds in robot XY.
static float robustWidthFromCluster(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cluster,
                                    const Calibration* calib,
                                    const WidthEstimationParams& widthParams) {
    if (!cluster || cluster->empty()) {
        return 0.0f;
    }
    std::vector<float> xs;
    std::vector<float> ys;
    xs.reserve(cluster->points.size());
    ys.reserve(cluster->points.size());
    const bool useRobot = (calib != nullptr && calib->valid);
    for (const auto& pt : cluster->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        if (useRobot) {
            Eigen::Vector3f p = transformPoint(*calib, Eigen::Vector3f(pt.x, pt.y, pt.z));
            xs.push_back(p.x());
            ys.push_back(p.y());
        } else {
            xs.push_back(pt.x);
            ys.push_back(pt.y);
        }
    }
    if (xs.empty() || ys.empty()) {
        return 0.0f;
    }
    float xLo = percentile(xs, widthParams.width_percentile_lo);
    float xHi = percentile(xs, widthParams.width_percentile_hi);
    float yLo = percentile(ys, widthParams.width_percentile_lo);
    float yHi = percentile(ys, widthParams.width_percentile_hi);
    float dx = xHi - xLo;
    float dy = yHi - yLo;
    return std::max(dx, dy);
}

static float medianOfDeque(const std::deque<float>& values) {
    if (values.empty()) {
        return 0.0f;
    }
    std::vector<float> tmp(values.begin(), values.end());
    std::sort(tmp.begin(), tmp.end());
    const std::size_t mid = tmp.size() / 2;
    if (tmp.size() % 2 == 1) {
        return tmp[mid];
    }
    return 0.5f * (tmp[mid - 1] + tmp[mid]);
}

static const TrackedObject* selectPickTarget(const std::vector<TrackedObject>& tracked) {
    const TrackedObject* best = nullptr;
    float bestScore = -1.0f;
    for (const auto& obj : tracked) {
        if (!obj.features.valid || !obj.id_assigned || !obj.seen_this_frame) {
            continue;
        }
        if (obj.state != TrackState::TRACKED) {
            continue;
        }
        Eigen::Vector3f size = getBboxSizeForTrack(obj);
        float width = std::max(size.x(), size.y());
        if (width <= 0.0f) {
            continue;
        }
        float score = width;
        if (score > bestScore) {
            bestScore = score;
            best = &obj;
        }
    }
    return best;
}

} // namespace

int runVisionViewer(const VisionViewerOptions& options) {
    const std::string& configPath = options.camera_config_path;
    PlaneSegmentationParams params = loadPlaneSegmentationParams(configPath);
    ClusterParams clusterParams = loadClusterParams(configPath);
    SceneSummaryConfig summaryConfig = loadSceneSummaryConfig(configPath);
    WidthEstimationParams widthParams = loadWidthEstimationParams(configPath);

    auto clusterWidthLabel = [&](bool from_cluster) -> std::string {
        if (!from_cluster) {
            return " (bbox)";
        }
        int pLo = static_cast<int>(std::lround(widthParams.width_percentile_lo * 100.0f));
        int pHi = static_cast<int>(std::lround(widthParams.width_percentile_hi * 100.0f));
        std::ostringstream oss;
        oss << " (cluster p" << pLo << "-p" << pHi << " robot, med5)";
        return oss.str();
    };
    const std::string modelLogDir = "logs/model_calls";
    const std::string modelImagesDir = joinPath(modelLogDir, "images");
    const std::string modelRequestsPath = joinPath(modelLogDir, "requests.jsonl");
    const std::string modelResponsesPath = joinPath(modelLogDir, "responses.jsonl");

    if (options.print_params) {
        std::cout << "Loaded segmentation params from " << configPath << std::endl;
        std::cout << "  distance_threshold_m=" << params.distance_threshold_m
                  << " max_iterations=" << params.max_iterations
                  << " use_axis_constraint=" << (params.use_axis_constraint ? "true" : "false")
                  << " axis=[" << params.axis.x() << ", " << params.axis.y() << ", " << params.axis.z() << "]"
                  << " eps_angle_deg=" << params.eps_angle_deg
                  << " min_inliers=" << params.min_inliers
                  << " min_inlier_ratio=" << params.min_inlier_ratio
                  << std::endl;
        std::cout << "Loaded clustering params from " << configPath << std::endl;
        std::cout << "  tolerance_m=" << clusterParams.tolerance_m
                  << " min_size=" << clusterParams.min_cluster_size
                  << " max_size=" << clusterParams.max_cluster_size
                  << " verbose=" << (clusterParams.verbose ? "true" : "false")
                  << std::endl;
        std::cout << "Loaded scene summary config from " << configPath << std::endl;
        std::cout << "  debug=" << (summaryConfig.debug ? "true" : "false")
                  << " log_path=" << summaryConfig.log_path
                  << " jsonl_path=" << summaryConfig.jsonl_path
                  << " annotated_dir=" << summaryConfig.annotated_dir
                  << " include_new_in_log=" << (summaryConfig.include_new_in_log ? "true" : "false")
                  << " include_lost_in_log=" << (summaryConfig.include_lost_in_log ? "true" : "false")
                  << " print_interval_sec=" << summaryConfig.print_interval_sec
                  << std::endl;
    }

    SceneSummaryOptions modelSummaryOptions;
    modelSummaryOptions.require_assigned_id = true;
    modelSummaryOptions.require_seen_this_frame = true;
    modelSummaryOptions.include_lost = false;
    modelSummaryOptions.use_smoothed = true;

    SceneSummaryOptions logSummaryOptions;
    logSummaryOptions.require_assigned_id = !summaryConfig.include_new_in_log;
    logSummaryOptions.require_seen_this_frame = !summaryConfig.include_lost_in_log;
    logSummaryOptions.include_lost = summaryConfig.include_lost_in_log;
    logSummaryOptions.use_smoothed = true;

    RobotConfig robotCfg;
    Pinch48dParams pinchParams;
    bool pickConfigReady = false;
    bool pickPreviewReady = false;
    bool pickExecuteReady = false;
    if (options.enable_pick_preview || options.enable_pick_execute) {
        robotCfg = loadRobotConfig(options.robot_config_path);
        pinchParams = loadPinch48dParams(options.primitives_config_path);
        pickConfigReady = robotCfg.valid;
        pickPreviewReady = options.enable_pick_preview && pickConfigReady;
        pickExecuteReady = options.enable_pick_execute && pickConfigReady;
        if (!pickConfigReady) {
            std::cerr << "Pick preview/execute disabled (robot config invalid): "
                      << options.robot_config_path << std::endl;
        }
    }

    OrbbecCamera cam;
    if (!cam.start()) {
        std::cerr << "Failed to start Orbbec camera" << std::endl;
        return 1;
    }

    SerialArduino arduino;
    bool arduinoConnected = false;
    JointAnglesDeg eepromJoints;
    int eepromGripper = pickConfigReady ? robotCfg.gripper.open : 0;
    bool haveEeprom = false;
    JointAnglesDeg readyJoints;
    readyJoints.base = static_cast<float>(kReadyBase);
    readyJoints.shoulder = static_cast<float>(kReadyShoulder);
    readyJoints.elbow = static_cast<float>(kReadyElbow);
    readyJoints.wrist = static_cast<float>(kReadyWrist);

    if (pickExecuteReady) {
        std::cout << "Connecting to Arduino..." << std::endl;
        arduinoConnected = arduino.connectAuto(SERIAL_BAUD_RATE);
        if (!arduinoConnected) {
            std::cerr << "Failed to connect to Arduino." << std::endl;
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        arduino.flushInput();

        if (!waitForArduinoReady(arduino, 3000)) {
            std::cerr << "Arduino did not respond to PING (continuing)." << std::endl;
        }

        for (int attempt = 0; attempt < 3 && !haveEeprom; ++attempt) {
            haveEeprom = readEepromAngles(arduino, eepromJoints, eepromGripper);
            if (!haveEeprom) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                arduino.flushInput();
            }
        }
        if (haveEeprom) {
            std::cout << "EEPROM angles: [base=" << eepromJoints.base
                      << ", shoulder=" << eepromJoints.shoulder
                      << ", elbow=" << eepromJoints.elbow
                      << ", wrist=" << eepromJoints.wrist
                      << ", gripper=" << eepromGripper << "]" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        } else {
            std::cout << "EEPROM angles read failed (continuing)." << std::endl;
        }

        std::cout << "Taking arm to Ready Position..." << std::endl;
        if (!sendMoveAndWait(arduino, robotCfg, readyJoints, robotCfg.gripper.open,
                             "ready", false)) {
            return 1;
        }
    }

    PclViewer viewer(options.window_title);
    viewer.setFlipXY(true);

    using Clock = std::chrono::steady_clock;

    DisplayMode mode = DisplayMode::Objects;
    bool showBoxes = options.show_boxes_default;
    bool requestPickPreview = false;
    bool requestNextTarget = false;
    bool requestPickExecute = false;
    bool requestPlaceBack = false;
    bool requestReadyPose = false;
    bool requestModelCapture = false;
    viewer.setKeyHandler([&](char key) {
        if (key == 't') {
            mode = nextMode(mode);
        }
        if (key == 'b') {
            showBoxes = !showBoxes;
        }
        if (key == 'k') requestPickPreview = true;
        if (key == 'm') requestNextTarget = true;
        if (key == 'd') requestPickExecute = true;
        if (key == 'i') requestPlaceBack = true;
        if (key == 'h') requestReadyPose = true;
        if (key == '.') requestModelCapture = true;
    });

    auto lastFpsTime = Clock::now();
    auto lastPrintTime = Clock::now();
    auto lastFeaturePrintTime = Clock::now();
    auto lastSummaryPrintTime = Clock::now();
    int frameCount = 0;
    double fps = 0.0;

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr lastRaw;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr lastObjects;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr lastPlane;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr lastClustersColored;
    std::size_t lastClusterCount = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> lastClusters;
    bool lastSuccess = false;
    cv::Mat lastColorBgr;
    OrbbecCamera::Intrinsics lastColorIntr;

    struct BoxCacheEntry {
        Eigen::Vector3f min;
        Eigen::Vector3f max;
        Clock::time_point last_seen;
    };
    std::unordered_map<std::string, BoxCacheEntry> boxCache;
    const double boxPersistSeconds = 2.0;

    TrackingParams trackingParams;
    trackingParams.max_match_distance_m = 0.05f;
    trackingParams.min_seen_frames = 5;
    trackingParams.max_missed_frames = 3;
    trackingParams.smoothing_alpha = 0.2f;
    trackingParams.verbose = false;
    ObjectTracker tracker(trackingParams);
    std::vector<TrackedObject> trackedObjects;
    std::vector<int> pickCandidates;
    int selectedCandidateIdx = -1;
    std::string selectedId;
    std::unordered_map<std::string, std::deque<float>> widthHistory;
    const std::size_t widthHistoryMax = 5;
    const int kMaxIntentClarify = 5;
    struct LastPickContext {
        bool valid = false;
        std::string id;
        Eigen::Vector3f robot_target = Eigen::Vector3f::Zero();
        float width_m = 0.0f;
    };
    LastPickContext lastPick;

    while (!viewer.isStopped()) {
        if (!viewer.isPaused()) {
            OrbbecCamera::FrameData frame;
            if (cam.grabFrames(frame, 1000)) {
                frameCount++;
                lastColorBgr = frame.colorBgr;
                lastColorIntr = cam.getColorIntrinsics();
                auto cloud = buildPointCloud(frame.depth16, frame.colorBgr,
                                             cam.getDepthIntrinsics(), frame.depthScale);
                if (cloud) {
                    PlaneSegmentationResult result = segmentTablePlane(cloud, params);
                    lastSuccess = result.success;
                    lastRaw = cloud;
                    lastObjects = result.objects;
                    lastPlane = result.plane;

                    ClusterResult clusterResult = clusterObjects(result.objects, clusterParams);
                    lastClustersColored = clusterResult.colored;
                    lastClusterCount = clusterResult.clusters.size();
                    lastClusters = clusterResult.clusters;

                    std::vector<ClusterFeatures> detFeatures;
                    detFeatures.reserve(lastClusters.size());
                    for (const auto& cluster : lastClusters) {
                        detFeatures.push_back(extractClusterFeatures(
                                cluster,
                                clusterParams.min_cluster_size,
                                clusterParams.max_cluster_size));
                    }
                    trackedObjects = tracker.update(detFeatures);

                    pickCandidates.clear();
                    pickCandidates.reserve(trackedObjects.size());
                    for (int i = 0; i < static_cast<int>(trackedObjects.size()); ++i) {
                        const auto& obj = trackedObjects[i];
                        if (!obj.features.valid || !obj.id_assigned || !obj.seen_this_frame) {
                            continue;
                        }
                        if (obj.state != TrackState::TRACKED) {
                            continue;
                        }
                        pickCandidates.push_back(i);
                    }
                    std::sort(pickCandidates.begin(), pickCandidates.end(),
                              [&](int a, int b) {
                                  return trackedObjects[a].id_num < trackedObjects[b].id_num;
                              });
                    if (pickCandidates.empty()) {
                        selectedCandidateIdx = -1;
                        selectedId.clear();
                    } else {
                        int foundIdx = -1;
                        if (!selectedId.empty()) {
                            for (int i = 0; i < static_cast<int>(pickCandidates.size()); ++i) {
                                if (trackedObjects[pickCandidates[i]].id == selectedId) {
                                    foundIdx = i;
                                    break;
                                }
                            }
                        }
                        if (foundIdx >= 0) {
                            selectedCandidateIdx = foundIdx;
                        } else {
                            selectedCandidateIdx = 0;
                            selectedId = trackedObjects[pickCandidates[0]].id;
                        }
                    }

                    // Update width history for the currently selected target (for smoothing).
                    if (selectedCandidateIdx >= 0 &&
                        selectedCandidateIdx < static_cast<int>(pickCandidates.size())) {
                        const auto& target = trackedObjects[pickCandidates[selectedCandidateIdx]];
                        if (target.last_det_idx >= 0 &&
                            target.last_det_idx < static_cast<int>(lastClusters.size())) {
                    auto cluster = lastClusters[target.last_det_idx];
                    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr full = cluster;
                    if (lastObjects) {
                        auto fullTmp = buildFullClusterForWidth(cluster, lastObjects, clusterParams, widthParams);
                        if (fullTmp && !fullTmp->empty()) {
                            full = fullTmp;
                        }
                    }
                    float width_m = robustWidthFromCluster(full, options.calibration, widthParams);
                    if (width_m > 0.0f) {
                        auto& hist = widthHistory[target.id];
                        hist.push_back(width_m);
                                if (hist.size() > widthHistoryMax) {
                                    hist.pop_front();
                                }
                            }
                        }
                    }

                    std::unordered_map<int, std::array<uint8_t, 3>> detColor;
                    for (const auto& obj : trackedObjects) {
                        if (obj.seen_this_frame && obj.last_det_idx >= 0 && obj.id_assigned) {
                            detColor[obj.last_det_idx] = {{obj.color_r, obj.color_g, obj.color_b}};
                        }
                    }

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(
                            new pcl::PointCloud<pcl::PointXYZRGB>());
                    for (std::size_t i = 0; i < lastClusters.size(); ++i) {
                        const auto& cluster = lastClusters[i];
                        if (!cluster || cluster->empty()) {
                            continue;
                        }
                        std::array<uint8_t, 3> color = {{160, 160, 160}};
                        auto it = detColor.find(static_cast<int>(i));
                        if (it != detColor.end()) {
                            color = it->second;
                        }
                        for (const auto& pt : cluster->points) {
                            pcl::PointXYZRGB cp = pt;
                            cp.r = color[0];
                            cp.g = color[1];
                            cp.b = color[2];
                            colored->points.push_back(cp);
                        }
                    }
                    colored->width = static_cast<uint32_t>(colored->points.size());
                    colored->height = 1;
                    colored->is_dense = true;
                    lastClustersColored = colored;

                    if (params.verbose && result.success) {
                        auto now = Clock::now();
                        std::chrono::duration<double> elapsed = now - lastPrintTime;
                        if (elapsed.count() >= 1.0) {
                            lastPrintTime = now;
                            if (result.coefficients && result.coefficients->values.size() >= 4) {
                                const auto& c = result.coefficients->values;
                                Eigen::Vector3f n(c[0], c[1], c[2]);
                                if (n.norm() > 1e-6f) {
                                    n.normalize();
                                }
                                std::cout << std::fixed << std::setprecision(4)
                                          << "Plane normal (camera frame): ["
                                          << n.x() << ", " << n.y() << ", " << n.z() << "]\n";
                            }
                        }
                    }
                }
            }
        }

        auto now = Clock::now();
        std::chrono::duration<double> elapsed = now - lastFpsTime;
        if (elapsed.count() >= 0.5) {
            fps = frameCount / elapsed.count();
            frameCount = 0;
            lastFpsTime = now;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr toShow;
        switch (mode) {
            case DisplayMode::Raw:
                toShow = lastRaw;
                break;
            case DisplayMode::Plane:
                toShow = lastPlane;
                break;
            case DisplayMode::Clusters:
                toShow = lastClustersColored;
                break;
            case DisplayMode::Objects:
            default:
                toShow = lastObjects;
                break;
        }
        if (!toShow || toShow->empty()) {
            toShow = lastRaw;
        }
        if (toShow) {
            viewer.updateCloud(toShow);
        }

        std::ostringstream stats;
        stats << "FPS " << std::fixed << std::setprecision(1) << fps;
        if (lastRaw) {
            stats << " | raw " << lastRaw->size();
        }
        if (lastPlane) {
            stats << " | plane " << lastPlane->size();
        }
        if (lastObjects) {
            stats << " | obj " << lastObjects->size();
        }
        stats << " | clusters " << lastClusterCount;
        if (!lastSuccess && lastRaw) {
            stats << " | plane not found";
        }

        std::string leftText;
        leftText = std::string("b - boxes (") + (showBoxes ? "on" : "off") + ")\n";
        leftText += "t - toggle view\n";
        leftText += stats.str();
        leftText += "\n";
        leftText += modeLabel(mode);

        std::string topLeftText = "period key (.) - model capture + call";

        std::string rightText;
        if (options.enable_pick_preview || options.enable_pick_execute) {
            rightText = "m - next target\n";
            if (options.enable_pick_preview) {
                rightText += "k - pick preview\n";
            }
            if (options.enable_pick_execute) {
                rightText += "d - execute pick\n";
                rightText += "i - place back\n";
                rightText += "h - ready pose\n";
                rightText += std::string("arm: ") + (arduinoConnected ? "connected\n" : "DISCONNECTED\n");
            }
            if (!selectedId.empty()) {
                rightText += "target: " + selectedId;
            }
        }
        if (options.show_calibration_status) {
            if (options.calibration && options.calibration->valid) {
                leftText += "\ncalib RMS ";
                std::ostringstream rms;
                rms << std::fixed << std::setprecision(3) << options.calibration->rms_error_m;
                leftText += rms.str();
            } else {
                leftText += "\ncalib MISSING";
            }
        }

        if (showBoxes) {
            auto nowCache = Clock::now();
            for (const auto& obj : trackedObjects) {
                if (obj.state != TrackState::TRACKED || !obj.features.valid ||
                    !obj.seen_this_frame || !obj.id_assigned) {
                    continue;
                }
                Eigen::Vector3f bmin = obj.has_smoothed ? obj.bbox_min_smoothed : obj.features.bbox_min;
                Eigen::Vector3f bmax = obj.has_smoothed ? obj.bbox_max_smoothed : obj.features.bbox_max;
                boxCache[obj.id] = {bmin, bmax, nowCache};
            }
            // Prune stale boxes.
            for (auto it = boxCache.begin(); it != boxCache.end(); ) {
                std::chrono::duration<double> age = nowCache - it->second.last_seen;
                if (age.count() > boxPersistSeconds) {
                    it = boxCache.erase(it);
                } else {
                    ++it;
                }
            }
        } else {
            boxCache.clear();
        }

        if (mode != DisplayMode::Plane && showBoxes && !boxCache.empty()) {
            std::vector<Eigen::Vector3f> mins;
            std::vector<Eigen::Vector3f> maxs;
            std::vector<std::string> labels;
            std::vector<Eigen::Vector3f> labelPos;
            mins.reserve(boxCache.size());
            maxs.reserve(boxCache.size());
            labels.reserve(boxCache.size());
            labelPos.reserve(boxCache.size());
            for (const auto& kv : boxCache) {
                mins.push_back(kv.second.min);
                maxs.push_back(kv.second.max);
                labels.push_back(kv.first);
                const float inset_x = 0.005f;
                const float inset_y = 0.015f;
                const float inset_z = 0.012f;
                labelPos.push_back(Eigen::Vector3f(kv.second.min.x() + inset_x,
                                                   kv.second.min.y() + inset_y,
                                                   kv.second.max.z() - inset_z));
            }
            viewer.updateBoundingBoxes(mins, maxs);
            viewer.updateBoxLabels(labels, labelPos);
        } else {
            viewer.updateBoundingBoxes({}, {});
            viewer.updateBoxLabels({}, {});
        }
        viewer.updateHud(leftText, rightText, topLeftText);
        viewer.spinOnce(10);

        if (summaryConfig.print_interval_sec > 0.0) {
            auto nowSummary = Clock::now();
            std::chrono::duration<double> elapsedSummary = nowSummary - lastSummaryPrintTime;
            if (elapsedSummary.count() >= summaryConfig.print_interval_sec) {
                lastSummaryPrintTime = nowSummary;

                SceneSummary logSummary = buildSceneSummary(trackedObjects, logSummaryOptions);
                if (logSummary.valid) {
                    if (summaryConfig.print_to_console) {
                        std::cout << "\n" << formatSceneSummary(logSummary) << std::endl;
                    }
                    appendSceneSummaryLog(logSummary, summaryConfig.log_path);
                    if (summaryConfig.print_to_console) {
                        std::string robotCentroids = formatRobotCentroids(logSummary, options.calibration);
                        if (!robotCentroids.empty()) {
                            std::cout << robotCentroids << std::endl;
                        }
                    }
                }

                SceneSummary modelSummary = buildSceneSummary(trackedObjects, modelSummaryOptions);
                if (modelSummary.valid) {
                    appendSceneSummaryJsonl(modelSummary, summaryConfig.jsonl_path);
                    if (summaryConfig.debug && !summaryConfig.annotated_dir.empty() &&
                        !lastColorBgr.empty()) {
                        CameraIntrinsics intr = toSceneIntrinsics(lastColorIntr);
                        std::string filename = "scene_" + timestampMsString() + ".png";
                        std::string outPath = joinPath(summaryConfig.annotated_dir, filename);
                        saveAnnotatedSceneFrame(modelSummary, lastColorBgr, intr, outPath);
                    }
                }
            }
        }

        auto computeWidthForTarget = [&](const TrackedObject& target,
                                         float& width_smoothed,
                                         bool& width_from_cluster) {
            float width_m = 0.0f;
            width_from_cluster = false;
            if (target.last_det_idx >= 0 &&
                target.last_det_idx < static_cast<int>(lastClusters.size())) {
                auto cluster = lastClusters[target.last_det_idx];
                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr full = cluster;
                if (lastObjects) {
                    auto fullTmp = buildFullClusterForWidth(cluster, lastObjects, clusterParams, widthParams);
                    if (fullTmp && !fullTmp->empty()) {
                        full = fullTmp;
                    }
                }
                width_m = robustWidthFromCluster(full, options.calibration, widthParams);
                width_from_cluster = (width_m > 0.0f);
            }
            if (!width_from_cluster) {
                Eigen::Vector3f size = getBboxSizeForTrack(target);
                width_m = std::max(size.x(), size.y());
            }

            width_smoothed = width_m;
            auto it = widthHistory.find(target.id);
            if (it != widthHistory.end() && !it->second.empty()) {
                width_smoothed = medianOfDeque(it->second);
            }
        };

        auto buildPickPlanForTarget = [&](const TrackedObject& target,
                                          PrimitivePlan& plan,
                                          float& width_smoothed,
                                          bool& width_from_cluster,
                                          Eigen::Vector3f& cam_centroid,
                                          Eigen::Vector3f& robot_centroid) -> bool {
            computeWidthForTarget(target, width_smoothed, width_from_cluster);

            cam_centroid = getCentroidForTrack(target);
            if (!options.calibration || !options.calibration->valid) {
                return false;
            }
            robot_centroid = transformPoint(*options.calibration, cam_centroid);
            plan = buildPinch48dPlan(robotCfg, pinchParams, robot_centroid, width_smoothed);
            return true;
        };

        auto findTrackedById = [&](const std::string& id) -> const TrackedObject* {
            for (const auto& obj : trackedObjects) {
                if (obj.id_assigned && obj.id == id) {
                    return &obj;
                }
            }
            return nullptr;
        };

        auto executePickForTarget = [&](const TrackedObject& target,
                                        bool promptConfirm) -> bool {
            if (!options.enable_pick_execute) {
                std::cout << "Pick execute disabled (enable in VisionViewerOptions)." << std::endl;
                return false;
            }
            if (!pickExecuteReady) {
                std::cout << "Pick execute unavailable (robot config invalid)." << std::endl;
                return false;
            }
            if (!arduinoConnected) {
                std::cout << "Pick execute unavailable (Arduino not connected)." << std::endl;
                return false;
            }
            if (!options.calibration || !options.calibration->valid) {
                std::cout << "Pick execute requires valid calibration.yaml (camera -> robot)." << std::endl;
                return false;
            }

            PrimitivePlan plan;
            float width_smoothed = 0.0f;
            bool width_from_cluster = false;
            Eigen::Vector3f cam_centroid = Eigen::Vector3f::Zero();
            Eigen::Vector3f robot_centroid = Eigen::Vector3f::Zero();
            if (!buildPickPlanForTarget(target, plan, width_smoothed,
                                        width_from_cluster, cam_centroid, robot_centroid)) {
                std::cout << "Pick execute requires valid calibration.yaml (camera -> robot)." << std::endl;
                return false;
            }

            std::cout << "\nExecute pick for " << target.id
                      << " | width_m=" << std::fixed << std::setprecision(3) << width_smoothed
                      << clusterWidthLabel(width_from_cluster)
                      << " | cam=(" << cam_centroid.x() << ", "
                      << cam_centroid.y() << ", " << cam_centroid.z() << ")"
                      << " | robot=(" << robot_centroid.x() << ", "
                      << robot_centroid.y() << ", " << robot_centroid.z() << ")"
                      << std::endl;

            if (!plan.success) {
                std::cout << "Plan failed: " << plan.reason << std::endl;
                return false;
            }

            if (promptConfirm) {
                std::cout << "This will move the robot. Proceed? (y/N): ";
                std::string confirm;
                std::getline(std::cin, confirm);
                if (confirm != "y" && confirm != "Y") {
                    std::cout << "Skipped sequence." << std::endl;
                    return false;
                }
            }

            bool pickOk = true;
            for (const auto& step : plan.steps) {
                if (!sendMoveAndWait(arduino, robotCfg, step.joints, step.gripper,
                                     step.name, pinchParams.enforce_workspace)) {
                    pickOk = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            if (pickOk) {
                lastPick.valid = true;
                lastPick.id = target.id;
                lastPick.robot_target = robot_centroid;
                lastPick.width_m = width_smoothed;
                std::cout << "Pick sequence complete." << std::endl;
            } else {
                std::cout << "Pick sequence aborted." << std::endl;
            }
            return pickOk;
        };

        if (requestModelCapture) {
            requestModelCapture = false;
            SceneSummary captureSummary = buildSceneSummary(trackedObjects, modelSummaryOptions);
            bool capture_ready = true;
            if (!captureSummary.valid) {
                capture_ready = false;
                std::cout << "Model capture note: no valid objects." << std::endl;
            }
            if (lastColorBgr.empty()) {
                capture_ready = false;
                std::cout << "Model capture note: no color frame available." << std::endl;
            }

            int64_t ts = timestampMs();
            std::string tsStr = std::to_string(ts);
            std::string imageName = "model_" + tsStr + ".png";
            std::string imagePath = joinPath(modelImagesDir, imageName);
            std::string tmpDir = joinPath(modelLogDir, "tmp");
            std::string tmpImagePath = joinPath(tmpDir, "model_" + tsStr + ".png");
            std::string tmpPath = joinPath(tmpDir, "request_" + tsStr + ".json");
            std::string jsonLine;
            bool okTmpJson = false;
            bool okTmpImg = false;
            if (capture_ready) {
                jsonLine = formatModelRequestJson(captureSummary, ts);
                CameraIntrinsics intr = toSceneIntrinsics(lastColorIntr);
                okTmpJson = writeFile(tmpPath, jsonLine);
                okTmpImg = saveAnnotatedSceneFrame(captureSummary, lastColorBgr, intr, tmpImagePath);
                if (!okTmpJson || !okTmpImg) {
                    capture_ready = false;
                    std::cout << "Model capture failed (temp_json=" << (okTmpJson ? "ok" : "fail")
                              << ", temp_image=" << (okTmpImg ? "ok" : "fail") << ")." << std::endl;
                }
            }

            {
                    auto runIntentCall = [&](const std::string& instruction,
                                             bool holding_item) -> std::optional<IntentDecision> {
                        const char* pyEnv = std::getenv("GRASP_MODEL_PYTHON");
                        std::string pythonExec = pyEnv ? pyEnv : "python3";
                        std::string cmd = "PYTHONPATH=python " + shellEscape(pythonExec) +
                                          " -m grasp_model.model_cli" +
                                          " --intent-only" +
                                          " --holding-item " + std::string(holding_item ? "true" : "false") +
                                          " --instruction " + shellEscape(instruction);
                        int exitCode = 0;
                        std::string output = runCommandCapture(cmd, exitCode);
                        std::string trimmed = trimWhitespace(output);
                        if (exitCode != 0 || trimmed.empty()) {
                            std::cout << "Intent call failed (exit=" << exitCode << ")." << std::endl;
                            if (!trimmed.empty()) {
                                std::cout << "Output: " << trimmed << std::endl;
                            }
                            return std::nullopt;
                        }
                        auto decision = parseIntentDecision(trimmed);
                        if (!decision) {
                            std::cout << "Intent response parse failed." << std::endl;
                            return std::nullopt;
                        }
                        return decision;
                    };

                    auto runModelCall = [&](const std::string& instruction,
                                            const std::vector<std::string>& history) -> std::optional<ModelDecision> {
                        const char* pyEnv = std::getenv("GRASP_MODEL_PYTHON");
                        std::string pythonExec = pyEnv ? pyEnv : "python3";
                        std::string historyPath = joinPath(tmpDir, "history_" + tsStr + ".json");
                        if (!history.empty()) {
                            std::ostringstream h;
                            h << "[";
                            for (std::size_t i = 0; i < history.size(); ++i) {
                                if (i > 0) h << ",";
                                h << "\"" << escapeJson(history[i]) << "\"";
                            }
                            h << "]";
                            writeFile(historyPath, h.str());
                        }
                        std::string cmd = "PYTHONPATH=python " + shellEscape(pythonExec) +
                                          " -m grasp_model.model_cli" +
                                          " --instruction " + shellEscape(instruction) +
                                          " --scene-json-file " + shellEscape(tmpPath) +
                                          " --image-path " + shellEscape(imagePath);
                        if (!history.empty()) {
                            cmd += " --history-file " + shellEscape(historyPath);
                        }
                        int exitCode = 0;
                        std::string output = runCommandCapture(cmd, exitCode);
                        if (!history.empty()) {
                            std::error_code ec;
                            std::filesystem::remove(historyPath, ec);
                        }
                        std::string trimmed = trimWhitespace(output);
                        if (exitCode != 0 || trimmed.empty()) {
                            std::cout << "Model call failed (exit=" << exitCode << ")." << std::endl;
                            if (!trimmed.empty()) {
                                std::cout << "Output: " << trimmed << std::endl;
                            }
                            return std::nullopt;
                        }
                        appendJsonlLine(modelResponsesPath, trimmed);
                        auto decision = parseModelDecision(trimmed);
                        if (!decision) {
                            std::cout << "Model response parse failed." << std::endl;
                            return std::nullopt;
                        }
                        std::cout << "\nModel: " << decision->reason << std::endl;
                        return decision;
                    };

                    auto runPickFlow = [&](const std::string& initial_instruction) {
                        std::string instruction = initial_instruction;
                        std::vector<std::string> history;
                        bool haveInstruction = true;
                        while (true) {
                            if (!haveInstruction) {
                                std::cout << "Enter instruction: " << std::flush;
                                std::getline(std::cin, instruction);
                                instruction = trimWhitespace(instruction);
                                if (instruction.empty()) {
                                    std::cout << "Model call skipped (empty instruction)." << std::endl;
                                    break;
                                }
                            }

                            auto decisionOpt = runModelCall(instruction, history);
                            haveInstruction = false;
                            if (!decisionOpt) {
                                break;
                            }
                            const ModelDecision& decision = *decisionOpt;

                            if (decision.confirm_needed || decision.target_is_null) {
                                if (!decision.reason.empty()) {
                                    std::cout << "\nModel: " << decision.reason << std::endl;
                                }
                                std::cout << "Enter refined instruction (empty to cancel): " << std::flush;
                                std::string refined;
                                std::getline(std::cin, refined);
                                refined = trimWhitespace(refined);
                                if (refined.empty()) {
                                    std::cout << "No refinement provided. Skipping." << std::endl;
                                    break;
                                }
                                history.push_back(instruction);
                                instruction = refined;
                                haveInstruction = true;
                                continue;
                            }

                            float preview_width = 0.0f;
                            bool preview_from_cluster = false;
                            const TrackedObject* previewTarget = findTrackedById(decision.target_id);
                            if (previewTarget) {
                                computeWidthForTarget(*previewTarget, preview_width, preview_from_cluster);
                            }
                            std::cout << "\nExecute pick for " << decision.target_id
                                      << "? [width_m=" << std::fixed << std::setprecision(3)
                                      << preview_width << "] (y/N): ";
                            std::string confirm;
                            std::getline(std::cin, confirm);
                            if (confirm == "y" || confirm == "Y") {
                                const TrackedObject* target = findTrackedById(decision.target_id);
                                if (!target) {
                                    std::cout << "Target not found: " << decision.target_id << std::endl;
                                } else {
                                    executePickForTarget(*target, false);
                                }
                                break;
                            }

                            std::cout << "Enter refined instruction (empty to cancel): " << std::flush;
                            std::string refined;
                            std::getline(std::cin, refined);
                            refined = trimWhitespace(refined);
                            if (refined.empty()) {
                                std::cout << "No refinement provided. Skipping." << std::endl;
                                break;
                            }
                            history.push_back(instruction);
                            instruction = refined;
                            haveInstruction = true;
                        }
                    };

                    bool tempImageMoved = false;
                    bool holding_item = lastPick.valid;
                    std::string instruction;
                    bool haveInstruction = false;
                    int clarifyCount = 0;
                    while (true) {
                        if (!haveInstruction) {
                            std::cout << "Enter instruction: " << std::flush;
                            std::getline(std::cin, instruction);
                            instruction = trimWhitespace(instruction);
                            if (instruction.empty()) {
                                std::cout << "Model call skipped (empty instruction)." << std::endl;
                                break;
                            }
                        }

                        auto intentOpt = runIntentCall(instruction, holding_item);
                        haveInstruction = false;
                        if (!intentOpt) {
                            break;
                        }
                        const IntentDecision& intent = *intentOpt;

                        if (intent.action == "clarify") {
                            if (clarifyCount >= kMaxIntentClarify) {
                                std::cout << "Too many clarification attempts. Skipping." << std::endl;
                                break;
                            }
                            clarifyCount++;
                            std::cout << "\nModel: " << intent.question << std::endl;
                            std::cout << "Enter clarification (empty to cancel): " << std::flush;
                            std::string refined;
                            std::getline(std::cin, refined);
                            refined = trimWhitespace(refined);
                            if (refined.empty()) {
                                std::cout << "No clarification provided. Skipping." << std::endl;
                                break;
                            }
                            instruction = refined;
                            haveInstruction = true;
                            continue;
                        }

                        if (intent.action == "place_back") {
                            requestPlaceBack = true;
                            break;
                        }

                        if (intent.action == "pick") {
                            if (!capture_ready) {
                                std::cout << "Pick unavailable (no valid capture)." << std::endl;
                                break;
                            }
                            if (!moveFile(tmpImagePath, imagePath)) {
                                std::cout << "Failed to move temp image to log: " << imagePath << std::endl;
                                break;
                            }
                            tempImageMoved = true;
                            if (!appendJsonlLine(modelRequestsPath, jsonLine)) {
                                std::cout << "Failed to append model request log." << std::endl;
                                break;
                            }
                            std::cout << "Saved model capture: " << imagePath << std::endl;
                            runPickFlow(instruction);
                            break;
                        }
                    }

                    std::error_code ec;
                    if (!tempImageMoved && !tmpImagePath.empty()) {
                        std::filesystem::remove(tmpImagePath, ec);
                    }
                    if (!tmpPath.empty()) {
                        std::filesystem::remove(tmpPath, ec);
                    }
            }
        }

        if (requestPickPreview) {
            requestPickPreview = false;
            if (!options.enable_pick_preview) {
                std::cout << "Pick preview disabled (enable in VisionViewerOptions)." << std::endl;
            } else if (!pickPreviewReady) {
                std::cout << "Pick preview unavailable (robot config invalid)." << std::endl;
            } else if (!options.calibration || !options.calibration->valid) {
                std::cout << "Pick preview requires valid calibration.yaml (camera -> robot)." << std::endl;
            } else {
                const TrackedObject* target = nullptr;
                if (selectedCandidateIdx >= 0 &&
                    selectedCandidateIdx < static_cast<int>(pickCandidates.size())) {
                    target = &trackedObjects[pickCandidates[selectedCandidateIdx]];
                }
                if (!target) {
                    std::cout << "Pick preview: no tracked target selected." << std::endl;
                } else {
                    PrimitivePlan plan;
                    float width_smoothed = 0.0f;
                    bool width_from_cluster = false;
                    Eigen::Vector3f cam_centroid = Eigen::Vector3f::Zero();
                    Eigen::Vector3f robot_centroid = Eigen::Vector3f::Zero();
                    if (!buildPickPlanForTarget(*target, plan, width_smoothed,
                                                width_from_cluster, cam_centroid, robot_centroid)) {
                        std::cout << "Pick preview requires valid calibration.yaml (camera -> robot)." << std::endl;
                        continue;
                    }
                    std::cout << "\nPick preview for " << target->id
                              << " | width_m=" << std::fixed << std::setprecision(3) << width_smoothed
                              << clusterWidthLabel(width_from_cluster)
                              << " | cam=(" << cam_centroid.x() << ", "
                              << cam_centroid.y() << ", " << cam_centroid.z() << ")"
                              << " | robot=(" << robot_centroid.x() << ", "
                              << robot_centroid.y() << ", " << robot_centroid.z() << ")"
                              << std::endl;
                    if (!plan.success) {
                        std::cout << "Plan failed: " << plan.reason << std::endl;
                    } else {
                        for (const auto& step : plan.steps) {
                            std::cout << "  - " << step.name
                                      << " | base=" << step.joints.base
                                      << " shoulder=" << step.joints.shoulder
                                      << " elbow=" << step.joints.elbow
                                      << " wrist=" << step.joints.wrist
                                      << " gripper=" << step.gripper
                                      << std::endl;
                        }
                    }
                }
            }
        }

        if (requestPickExecute) {
            requestPickExecute = false;
            const TrackedObject* target = nullptr;
            if (selectedCandidateIdx >= 0 &&
                selectedCandidateIdx < static_cast<int>(pickCandidates.size())) {
                target = &trackedObjects[pickCandidates[selectedCandidateIdx]];
            }
            if (!target) {
                std::cout << "Pick execute: no tracked target selected." << std::endl;
            } else {
                executePickForTarget(*target, true);
            }
        }

        if (requestReadyPose) {
            requestReadyPose = false;
            if (!options.enable_pick_execute) {
                std::cout << "Ready pose disabled (enable in VisionViewerOptions)." << std::endl;
            } else if (!pickExecuteReady) {
                std::cout << "Ready pose unavailable (robot config invalid)." << std::endl;
            } else if (!arduinoConnected) {
                std::cout << "Ready pose unavailable (Arduino not connected)." << std::endl;
            } else {
                std::cout << "Moving to Ready Position..." << std::endl;
                if (!sendMoveAndWait(arduino, robotCfg, readyJoints, robotCfg.gripper.open,
                                     "ready", false)) {
                    std::cout << "Ready move failed." << std::endl;
                }
            }
        }

        if (requestPlaceBack) {
            requestPlaceBack = false;
            if (!options.enable_pick_execute) {
                std::cout << "Place back disabled (enable in VisionViewerOptions)." << std::endl;
            } else if (!pickExecuteReady) {
                std::cout << "Place back unavailable (robot config invalid)." << std::endl;
            } else if (!arduinoConnected) {
                std::cout << "Place back unavailable (Arduino not connected)." << std::endl;
            } else if (!lastPick.valid) {
                std::cout << "Place back unavailable (no successful pick on record)." << std::endl;
            } else {
                PrimitivePlan pickPlan = buildPinch48dPlan(robotCfg, pinchParams,
                                                           lastPick.robot_target,
                                                           lastPick.width_m);
                if (!pickPlan.success) {
                    std::cout << "Place back plan failed: " << pickPlan.reason << std::endl;
                } else if (pickPlan.steps.size() < 4) {
                    std::cout << "Place back plan invalid (missing steps)." << std::endl;
                } else {
                    const int gripper_open = pickPlan.steps[0].gripper;
                    const int gripper_close = pickPlan.steps[2].gripper;
                    PrimitivePlan placePlan;
                    placePlan.success = true;
                    placePlan.steps.push_back({"approach_hold", pickPlan.steps[0].joints, gripper_close});
                    placePlan.steps.push_back({"descend_hold", pickPlan.steps[1].joints, gripper_close});
                    placePlan.steps.push_back({"open", pickPlan.steps[1].joints, gripper_open});
                    placePlan.steps.push_back({"lift_open", pickPlan.steps[3].joints, gripper_open});

                    std::cout << "\nPlace back to last pick (" << lastPick.id
                              << ") | robot=(" << lastPick.robot_target.x() << ", "
                              << lastPick.robot_target.y() << ", " << lastPick.robot_target.z() << ")"
                              << std::endl;

                    std::cout << "This will move the robot. Proceed? (y/N): ";
                    std::string confirm;
                    std::getline(std::cin, confirm);
                    if (confirm != "y" && confirm != "Y") {
                        std::cout << "Skipped sequence." << std::endl;
                        continue;
                    }

                    bool placeOk = true;
                    for (const auto& step : placePlan.steps) {
                        if (!sendMoveAndWait(arduino, robotCfg, step.joints, step.gripper,
                                             step.name, pinchParams.enforce_workspace)) {
                            placeOk = false;
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    if (placeOk) {
                        lastPick.valid = false;
                        std::cout << "Place sequence complete." << std::endl;
                    } else {
                        std::cout << "Place sequence aborted." << std::endl;
                    }
                }
            }
        }

        if (requestNextTarget) {
            requestNextTarget = false;
            if (pickCandidates.empty()) {
                std::cout << "No tracked targets to select." << std::endl;
            } else {
                if (selectedCandidateIdx < 0) {
                    selectedCandidateIdx = 0;
                } else {
                    selectedCandidateIdx = (selectedCandidateIdx + 1) %
                                           static_cast<int>(pickCandidates.size());
                }
                selectedId = trackedObjects[pickCandidates[selectedCandidateIdx]].id;
                std::cout << "Selected target: " << selectedId << std::endl;
            }
        }

        if (options.print_feature_stats &&
            mode == DisplayMode::Clusters && !lastClusters.empty() && lastObjects) {
            auto nowPrint = Clock::now();
            std::chrono::duration<double> elapsedPrint = nowPrint - lastFeaturePrintTime;
            if (elapsedPrint.count() >= 5.0) {
                lastFeaturePrintTime = nowPrint;
                std::vector<ClusterFeatures> features;
                features.reserve(lastClusters.size());
                float pad = std::max(clusterParams.tolerance_m, clusterParams.voxel_leaf_m);

                for (const auto& cluster : lastClusters) {
                    if (!cluster || cluster->empty()) {
                        features.push_back(ClusterFeatures());
                        continue;
                    }
                    float minX = std::numeric_limits<float>::infinity();
                    float minY = std::numeric_limits<float>::infinity();
                    float minZ = std::numeric_limits<float>::infinity();
                    float maxX = -std::numeric_limits<float>::infinity();
                    float maxY = -std::numeric_limits<float>::infinity();
                    float maxZ = -std::numeric_limits<float>::infinity();
                    for (const auto& pt : cluster->points) {
                        if (!pcl::isFinite(pt)) {
                            continue;
                        }
                        minX = std::min(minX, pt.x);
                        minY = std::min(minY, pt.y);
                        minZ = std::min(minZ, pt.z);
                        maxX = std::max(maxX, pt.x);
                        maxY = std::max(maxY, pt.y);
                        maxZ = std::max(maxZ, pt.z);
                    }
                    minX -= pad; minY -= pad; minZ -= pad;
                    maxX += pad; maxY += pad; maxZ += pad;

                    Eigen::Vector3f centroid_ds = Eigen::Vector3f::Zero();
                    std::size_t dsCount = 0;
                    for (const auto& pt : cluster->points) {
                        if (!pcl::isFinite(pt)) {
                            continue;
                        }
                        centroid_ds += Eigen::Vector3f(pt.x, pt.y, pt.z);
                        dsCount++;
                    }
                    if (dsCount > 0) {
                        centroid_ds /= static_cast<float>(dsCount);
                    }
                    float radius = std::max(0.03f, clusterParams.tolerance_m * 1.5f);
                    float radius2 = radius * radius;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullCluster(
                            new pcl::PointCloud<pcl::PointXYZRGB>());
                    fullCluster->points.reserve(cluster->points.size() * 4);
                    for (const auto& pt : lastObjects->points) {
                        if (!pcl::isFinite(pt)) {
                            continue;
                        }
                        if (pt.x < minX || pt.x > maxX) continue;
                        if (pt.y < minY || pt.y > maxY) continue;
                        if (pt.z < minZ || pt.z > maxZ) continue;
                        Eigen::Vector3f p(pt.x, pt.y, pt.z);
                        if ((p - centroid_ds).squaredNorm() > radius2) {
                            continue;
                        }
                        fullCluster->points.push_back(pt);
                    }
                    fullCluster->width = static_cast<uint32_t>(fullCluster->points.size());
                    fullCluster->height = 1;
                    fullCluster->is_dense = true;

                    ClusterFeatures f = extractClusterFeatures(
                            fullCluster,
                            clusterParams.min_cluster_size,
                            clusterParams.max_cluster_size);
                    f.confidence = std::min(1.0f, static_cast<float>(cluster->size()) /
                                                     static_cast<float>(clusterParams.max_cluster_size));
                    features.push_back(f);
                }
                std::size_t printed = 0;
                std::cout << "\nTracked features:\n";
                for (const auto& obj : trackedObjects) {
                    if (!obj.seen_this_frame || obj.last_det_idx < 0 || !obj.id_assigned) {
                        continue;
                    }
                    if (static_cast<std::size_t>(obj.last_det_idx) >= features.size()) {
                        continue;
                    }
                    const auto& f = features[static_cast<std::size_t>(obj.last_det_idx)];
                    if (!f.valid) {
                        continue;
                    }
                    std::cout << "  [" << obj.id << "] points=" << f.point_count
                              << " centroid=(" << std::fixed << std::setprecision(3)
                              << f.centroid.x() << ", " << f.centroid.y() << ", " << f.centroid.z() << ")"
                              << " size=(" << f.bbox_size.x() << ", " << f.bbox_size.y() << ", " << f.bbox_size.z() << ")"
                              << " aspect ratios=(xy:" << std::setprecision(2)
                              << f.aspect_xy << ", xz:" << f.aspect_xz << ", yz:" << f.aspect_yz << ")"
                              << " median_rgb=(" << std::setprecision(1)
                              << f.median_rgb.x() << ", " << f.median_rgb.y() << ", " << f.median_rgb.z() << ")"
                              << " conf=" << std::setprecision(2) << f.confidence
                              << std::endl;
                    printed++;
                }
                if (printed == 0) {
                    std::cout << "  (no tracked objects this frame)" << std::endl;
                }
            }
        }
    }

    if (pickExecuteReady && arduinoConnected) {
        std::cout << "Returning to Ready Position..." << std::endl;
        if (sendMoveAndWait(arduino, robotCfg, readyJoints, robotCfg.gripper.open,
                            "ready", false)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }

        if (haveEeprom) {
            std::cout << "Returning to EEPROM angles..." << std::endl;
            if (!sendMoveAndWait(arduino, robotCfg, eepromJoints, eepromGripper,
                                 "eeprom_restore", false)) {
                std::cerr << "Failed to return to EEPROM angles." << std::endl;
            }
        } else {
            std::cout << "No EEPROM angles available; leaving arm at Ready Position." << std::endl;
        }
    }

    return 0;
}
