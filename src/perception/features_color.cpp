#include "grasp_system/perception/features_color.h"

#include <pcl/common/point_tests.h>
#include <algorithm>
#include <limits>

namespace {

float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

float computeConfidence(std::size_t points, int min_points, int max_points) {
    if (max_points <= min_points) {
        return 1.0f;
    }
    float numer = static_cast<float>(points - static_cast<std::size_t>(min_points));
    float denom = static_cast<float>(max_points - min_points);
    return clamp01(numer / denom);
}

} // namespace

ClusterFeatures extractClusterFeatures(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cluster,
        int min_points,
        int max_points) {
    ClusterFeatures features;
    if (!cluster || cluster->empty()) {
        return features;
    }

    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    std::vector<uint8_t> rs;
    std::vector<uint8_t> gs;
    std::vector<uint8_t> bs;

    std::size_t count = 0;
    for (const auto& pt : cluster->points) {
        if (!pcl::isFinite(pt)) {
            continue;
        }
        count++;
        sumX += pt.x;
        sumY += pt.y;
        sumZ += pt.z;
        rs.push_back(pt.r);
        gs.push_back(pt.g);
        bs.push_back(pt.b);

        minX = std::min(minX, pt.x);
        minY = std::min(minY, pt.y);
        minZ = std::min(minZ, pt.z);
        maxX = std::max(maxX, pt.x);
        maxY = std::max(maxY, pt.y);
        maxZ = std::max(maxZ, pt.z);
    }

    if (count == 0) {
        return features;
    }

    features.valid = true;
    features.point_count = count;
    features.centroid = Eigen::Vector3f(
            static_cast<float>(sumX / count),
            static_cast<float>(sumY / count),
            static_cast<float>(sumZ / count));
    features.bbox_min = Eigen::Vector3f(minX, minY, minZ);
    features.bbox_max = Eigen::Vector3f(maxX, maxY, maxZ);
    features.bbox_size = features.bbox_max - features.bbox_min;
    auto medianChannel = [](std::vector<uint8_t>& v) -> float {
        if (v.empty()) {
            return 0.0f;
        }
        std::size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        float med = static_cast<float>(v[mid]);
        if (v.size() % 2 == 0) {
            auto maxLower = *std::max_element(v.begin(), v.begin() + mid);
            med = 0.5f * (med + static_cast<float>(maxLower));
        }
        return med;
    };

    features.median_rgb = Eigen::Vector3f(
            medianChannel(rs),
            medianChannel(gs),
            medianChannel(bs));

    if (features.bbox_size.y() != 0.0f) {
        features.aspect_xy = features.bbox_size.x() / features.bbox_size.y();
    }
    if (features.bbox_size.z() != 0.0f) {
        features.aspect_xz = features.bbox_size.x() / features.bbox_size.z();
    }
    if (features.bbox_size.z() != 0.0f) {
        features.aspect_yz = features.bbox_size.y() / features.bbox_size.z();
    }

    features.confidence = computeConfidence(count, min_points, max_points);
    return features;
}

std::vector<ClusterFeatures> extractClusterFeatures(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clusters,
        int min_points,
        int max_points) {
    std::vector<ClusterFeatures> out;
    out.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        out.push_back(extractClusterFeatures(cluster, min_points, max_points));
    }
    return out;
}
