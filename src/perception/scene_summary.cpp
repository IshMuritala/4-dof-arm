#include "grasp_system/perception/scene_summary.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

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

bool ensureParentDir(const std::string& path) {
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

int64_t nowEpochMs() {
    using namespace std::chrono;
    auto now = system_clock::now();
    return duration_cast<milliseconds>(now.time_since_epoch()).count();
}

std::string escapeJson(const std::string& s) {
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

bool projectPoint(const CameraIntrinsics& intr,
                  const Eigen::Vector3f& p,
                  cv::Point& out) {
    if (intr.fx == 0.0f || intr.fy == 0.0f || intr.width <= 0 || intr.height <= 0) {
        return false;
    }
    if (p.z() <= 0.0f) {
        return false;
    }
    float u = intr.fx * p.x() / p.z() + intr.cx;
    float v = intr.fy * p.y() / p.z() + intr.cy;
    if (u < 0.0f || v < 0.0f ||
        u >= static_cast<float>(intr.width) ||
        v >= static_cast<float>(intr.height)) {
        return false;
    }
    out = cv::Point(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
    return true;
}

bool projectBboxToRect(const CameraIntrinsics& intr,
                       const Eigen::Vector3f& bmin,
                       const Eigen::Vector3f& bmax,
                       cv::Rect& out) {
    if (intr.width <= 0 || intr.height <= 0) {
        return false;
    }
    std::vector<cv::Point> pts;
    pts.reserve(8);
    const float xs[2] = {bmin.x(), bmax.x()};
    const float ys[2] = {bmin.y(), bmax.y()};
    const float zs[2] = {bmin.z(), bmax.z()};
    for (float x : xs) {
        for (float y : ys) {
            for (float z : zs) {
                cv::Point p;
                if (projectPoint(intr, Eigen::Vector3f(x, y, z), p)) {
                    pts.push_back(p);
                }
            }
        }
    }
    if (pts.size() < 2) {
        return false;
    }
    int minx = pts[0].x;
    int maxx = pts[0].x;
    int miny = pts[0].y;
    int maxy = pts[0].y;
    for (const auto& p : pts) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }
    minx = std::max(0, std::min(minx, intr.width - 1));
    maxx = std::max(0, std::min(maxx, intr.width - 1));
    miny = std::max(0, std::min(miny, intr.height - 1));
    maxy = std::max(0, std::min(maxy, intr.height - 1));
    if (maxx <= minx || maxy <= miny) {
        return false;
    }
    out = cv::Rect(cv::Point(minx, miny), cv::Point(maxx, maxy));
    return true;
}

} // namespace

SceneSummary buildSceneSummary(const std::vector<ClusterFeatures>& features,
                               const std::string& id_prefix) {
    SceneSummary summary;
    if (features.empty()) {
        return summary;
    }
    summary.objects.reserve(features.size());
    int id_idx = 0;
    for (const auto& f : features) {
        if (!f.valid) {
            continue;
        }
        ObjectInfo obj;
        obj.id = id_prefix + "_" + std::to_string(id_idx++);
        obj.features = f;
        obj.state = TrackState::TRACKED;
        obj.seen_this_frame = true;
        obj.id_assigned = true;
        summary.objects.push_back(obj);
    }
    summary.valid = !summary.objects.empty();
    return summary;
}

SceneSummary buildSceneSummary(const std::vector<TrackedObject>& tracks,
                               const SceneSummaryOptions& options) {
    SceneSummary summary;
    if (tracks.empty()) {
        return summary;
    }
    summary.objects.reserve(tracks.size());
    int pending_idx = 0;
    for (const auto& track : tracks) {
        if (!track.features.valid) {
            continue;
        }
        if (options.require_seen_this_frame && !track.seen_this_frame) {
            continue;
        }
        if (!options.include_lost && track.state == TrackState::LOST) {
            continue;
        }
        if (options.require_assigned_id && !track.id_assigned) {
            continue;
        }
        ObjectInfo obj;
        if (track.id_assigned) {
            obj.id = track.id;
        } else {
            obj.id = "pending_" + std::to_string(pending_idx++);
        }
        obj.features = track.features;
        if (options.use_smoothed && track.has_smoothed) {
            obj.features.centroid = track.centroid_smoothed;
            obj.features.bbox_min = track.bbox_min_smoothed;
            obj.features.bbox_max = track.bbox_max_smoothed;
            obj.features.bbox_size = obj.features.bbox_max - obj.features.bbox_min;
        }
        obj.state = track.state;
        obj.seen_this_frame = track.seen_this_frame;
        obj.id_assigned = track.id_assigned;
        summary.objects.push_back(obj);
    }
    summary.valid = !summary.objects.empty();
    return summary;
}

std::string formatSceneSummary(const SceneSummary& summary, int precision) {
    std::ostringstream oss;
    const int64_t ms = nowEpochMs();
    oss << "SceneSummary [ms=" << ms << "] (" << summary.objects.size() << " objects)";
    if (!summary.valid) {
        return oss.str();
    }
    oss << std::fixed << std::setprecision(precision);
    for (const auto& obj : summary.objects) {
        const auto& f = obj.features;
        oss << "\n  " << obj.id;
        if (!obj.id_assigned) {
            oss << " (pending)";
        } else if (obj.state == TrackState::NEW) {
            oss << " (new)";
        } else if (obj.state == TrackState::LOST) {
            oss << " (lost)";
        }
        oss << ": points=" << f.point_count
            << " centroid=(" << f.centroid.x() << ", " << f.centroid.y() << ", " << f.centroid.z() << ")"
            << " size=(" << f.bbox_size.x() << ", " << f.bbox_size.y() << ", " << f.bbox_size.z() << ")"
            << " median_rgb=(" << f.median_rgb.x() << ", " << f.median_rgb.y() << ", " << f.median_rgb.z() << ")"
            << " aspect_xy=" << f.aspect_xy
            << " aspect_xz=" << f.aspect_xz
            << " aspect_yz=" << f.aspect_yz
            << " conf=" << f.confidence;
    }
    return oss.str();
}

std::string formatSceneSummaryJson(const SceneSummary& summary, int precision) {
    std::ostringstream oss;
    const int64_t ms = nowEpochMs();
    oss << std::fixed << std::setprecision(precision);
    oss << "{";
    oss << "\"timestamp_ms\":" << ms << ",";
    oss << "\"valid\":" << (summary.valid ? "true" : "false") << ",";
    oss << "\"object_count\":" << summary.objects.size() << ",";
    oss << "\"objects\":[";
    for (std::size_t i = 0; i < summary.objects.size(); ++i) {
        const auto& obj = summary.objects[i];
        const auto& f = obj.features;
        if (i > 0) {
            oss << ",";
        }
        oss << "{";
        oss << "\"id\":\"" << escapeJson(obj.id) << "\",";
        oss << "\"point_count\":" << f.point_count << ",";
        oss << "\"centroid\":[" << f.centroid.x() << "," << f.centroid.y() << "," << f.centroid.z() << "],";
        oss << "\"bbox_min\":[" << f.bbox_min.x() << "," << f.bbox_min.y() << "," << f.bbox_min.z() << "],";
        oss << "\"bbox_max\":[" << f.bbox_max.x() << "," << f.bbox_max.y() << "," << f.bbox_max.z() << "],";
        oss << "\"bbox_size\":[" << f.bbox_size.x() << "," << f.bbox_size.y() << "," << f.bbox_size.z() << "],";
        oss << "\"median_rgb\":[" << f.median_rgb.x() << "," << f.median_rgb.y() << "," << f.median_rgb.z() << "],";
        oss << "\"aspect_xy\":" << f.aspect_xy << ",";
        oss << "\"aspect_xz\":" << f.aspect_xz << ",";
        oss << "\"aspect_yz\":" << f.aspect_yz << ",";
        oss << "\"confidence\":" << f.confidence;
        oss << "}";
    }
    oss << "]";
    oss << "}";
    return oss.str();
}

bool appendSceneSummaryLog(const SceneSummary& summary, const std::string& path) {
    if (!summary.valid || path.empty()) {
        return false;
    }
    if (!ensureParentDir(path)) {
        return false;
    }
    std::ofstream out(path, std::ios::app);
    if (!out) {
        return false;
    }
    out << formatSceneSummary(summary) << "\n";
    return true;
}

bool appendSceneSummaryJsonl(const SceneSummary& summary, const std::string& path) {
    if (!summary.valid || path.empty()) {
        return false;
    }
    if (!ensureParentDir(path)) {
        return false;
    }
    std::ofstream out(path, std::ios::app);
    if (!out) {
        return false;
    }
    out << formatSceneSummaryJson(summary) << "\n";
    return true;
}

SceneSummaryConfig loadSceneSummaryConfig(const std::string& yamlPath) {
    SceneSummaryConfig cfg;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "loadSceneSummaryConfig: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return cfg;
    }
    if (!fs.isOpened()) {
        return cfg;
    }
    const cv::FileNode node = fs["scene_summary"];
    if (node.empty()) {
        return cfg;
    }
    readNodeIfPresent(node, "debug", cfg.debug);
    readNodeIfPresent(node, "log_path", cfg.log_path);
    readNodeIfPresent(node, "jsonl_path", cfg.jsonl_path);
    readNodeIfPresent(node, "annotated_dir", cfg.annotated_dir);
    readNodeIfPresent(node, "include_new_in_log", cfg.include_new_in_log);
    readNodeIfPresent(node, "include_lost_in_log", cfg.include_lost_in_log);
    readNodeIfPresent(node, "print_to_console", cfg.print_to_console);
    readNodeIfPresent(node, "print_interval_sec", cfg.print_interval_sec);
    return cfg;
}

bool annotateSceneSummary(const SceneSummary& summary,
                          const cv::Mat& colorBgr,
                          const CameraIntrinsics& intr,
                          cv::Mat& outAnnotated) {
    if (!summary.valid || colorBgr.empty()) {
        return false;
    }
    outAnnotated = colorBgr.clone();
    for (const auto& obj : summary.objects) {
        cv::Point px;
        if (!projectPoint(intr, obj.features.centroid, px)) {
            continue;
        }
        const auto& rgb = obj.features.median_rgb;
        cv::Scalar color(rgb.z(), rgb.y(), rgb.x());
        cv::Rect rect;
        if (projectBboxToRect(intr, obj.features.bbox_min, obj.features.bbox_max, rect)) {
            cv::rectangle(outAnnotated, rect, color, 2, cv::LINE_AA);
        }
        cv::circle(outAnnotated, px, 4, color, -1, cv::LINE_AA);
        cv::Point labelPos(px.x + 6, px.y - 6);
        cv::putText(outAnnotated, obj.id, labelPos, cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
        cv::putText(outAnnotated, obj.id, labelPos, cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    return true;
}

bool saveAnnotatedSceneFrame(const SceneSummary& summary,
                             const cv::Mat& colorBgr,
                             const CameraIntrinsics& intr,
                             const std::string& outPath) {
    if (outPath.empty()) {
        return false;
    }
    cv::Mat annotated;
    if (!annotateSceneSummary(summary, colorBgr, intr, annotated)) {
        return false;
    }
    if (!ensureParentDir(outPath)) {
        return false;
    }
    return cv::imwrite(outPath, annotated);
}
