#include "grasp_system/perception/segmentation.h"

#include <pcl/common/angles.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <array>
#include <iostream>
#include <vector>

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

void readNodeIfPresent(const cv::FileNode& node, const char* key, std::size_t& value) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        int64_t tmp = 0;
        child >> tmp;
        if (tmp >= 0) {
            value = static_cast<std::size_t>(tmp);
        }
    }
}

Eigen::Vector3f readAxisIfPresent(const cv::FileNode& node, const char* key, const Eigen::Vector3f& fallback) {
    const cv::FileNode axisNode = node[key];
    if (axisNode.empty() || axisNode.type() != cv::FileNode::SEQ || axisNode.size() != 3) {
        return fallback;
    }
    std::vector<float> vals;
    axisNode >> vals;
    if (vals.size() != 3) {
        return fallback;
    }
    return Eigen::Vector3f(vals[0], vals[1], vals[2]);
}

} // namespace

PlaneSegmentationParams loadPlaneSegmentationParams(const std::string& yamlPath) {
    PlaneSegmentationParams params;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "loadPlaneSegmentationParams: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return params;
    }
    if (!fs.isOpened()) {
        return params;
    }
    const cv::FileNode seg = fs["segmentation"];
    if (seg.empty()) {
        return params;
    }
    readNodeIfPresent(seg, "plane_distance_threshold_m", params.distance_threshold_m);
    readNodeIfPresent(seg, "plane_max_iterations", params.max_iterations);
    readNodeIfPresent(seg, "plane_use_axis_constraint", params.use_axis_constraint);
    params.axis = readAxisIfPresent(seg, "plane_axis", params.axis);
    readNodeIfPresent(seg, "plane_eps_angle_deg", params.eps_angle_deg);
    readNodeIfPresent(seg, "plane_min_inliers", params.min_inliers);
    readNodeIfPresent(seg, "plane_min_inlier_ratio", params.min_inlier_ratio);
    readNodeIfPresent(seg, "plane_verbose", params.verbose);
    return params;
}

ClusterParams loadClusterParams(const std::string& yamlPath) {
    ClusterParams params;
    cv::FileStorage fs;
    try {
        fs.open(yamlPath, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        std::cerr << "loadClusterParams: failed to open " << yamlPath
                  << " (" << e.what() << ")" << std::endl;
        return params;
    }
    if (!fs.isOpened()) {
        return params;
    }
    const cv::FileNode node = fs["clustering"];
    if (node.empty()) {
        return params;
    }
    readNodeIfPresent(node, "cluster_tolerance_m", params.tolerance_m);
    readNodeIfPresent(node, "cluster_min_size", params.min_cluster_size);
    readNodeIfPresent(node, "cluster_max_size", params.max_cluster_size);
    readNodeIfPresent(node, "cluster_voxel_leaf_m", params.voxel_leaf_m);
    readNodeIfPresent(node, "cluster_merge_distance_m", params.merge_distance_m);
    readNodeIfPresent(node, "cluster_min_bbox_size_m", params.min_bbox_size_m);
    readNodeIfPresent(node, "cluster_verbose", params.verbose);
    return params;
}

PlaneSegmentationResult segmentTablePlane(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
        const PlaneSegmentationParams& params) {
    PlaneSegmentationResult result;
    result.objects.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    result.plane.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    result.coefficients.reset(new pcl::ModelCoefficients());
    result.inliers.reset(new pcl::PointIndices());

    if (!input || input->empty()) {
        return result;
    }

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params.distance_threshold_m);
    seg.setMaxIterations(params.max_iterations);

    if (params.use_axis_constraint) {
        Eigen::Vector3f axis = params.axis;
        if (axis.norm() > 1e-6f) {
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            axis.normalize();
            seg.setAxis(axis);
            seg.setEpsAngle(pcl::deg2rad(params.eps_angle_deg));
        } else {
            if (params.verbose) {
                std::cerr << "segmentTablePlane: axis constraint enabled but axis is zero; "
                          << "falling back to unconstrained plane." << std::endl;
            }
            seg.setModelType(pcl::SACMODEL_PLANE);
        }
    } else {
        seg.setModelType(pcl::SACMODEL_PLANE);
    }

    seg.setInputCloud(input);
    seg.segment(*result.inliers, *result.coefficients);

    const std::size_t total = input->size();
    const std::size_t inlierCount = result.inliers->indices.size();
    const std::size_t minByRatio =
            static_cast<std::size_t>(params.min_inlier_ratio * static_cast<float>(total));
    const std::size_t required = std::max(params.min_inliers, minByRatio);

    if (inlierCount < required) {
        if (params.verbose) {
            std::cerr << "segmentTablePlane: inliers " << inlierCount
                      << " below required " << required << std::endl;
        }
        *result.objects = *input;
        return result;
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(input);
    extract.setIndices(result.inliers);

    extract.setNegative(false);
    extract.filter(*result.plane);

    extract.setNegative(true);
    extract.filter(*result.objects);

    result.success = true;
    return result;
}

ClusterResult clusterObjects(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
        const ClusterParams& params) {
    ClusterResult result;
    result.colored.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (!input || input->empty()) {
        return result;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());
    if (params.voxel_leaf_m > 0.0f) {
        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        vox.setLeafSize(params.voxel_leaf_m, params.voxel_leaf_m, params.voxel_leaf_m);
        vox.setInputCloud(input);
        vox.filter(*clusterCloud);
        if (clusterCloud->empty()) {
            *clusterCloud = *input;
        }
    } else {
        *clusterCloud = *input;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(clusterCloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(params.tolerance_m);
    ec.setMinClusterSize(params.min_cluster_size);
    ec.setMaxClusterSize(params.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(clusterCloud);
    ec.extract(clusterIndices);

    if (params.verbose) {
        std::cout << "clusterObjects: clusters=" << clusterIndices.size()
                  << " tolerance=" << params.tolerance_m
                  << " min=" << params.min_cluster_size
                  << " max=" << params.max_cluster_size << std::endl;
    }

    static const std::array<std::array<uint8_t, 3>, 12> palette = {{
        {{255,  80,  80}},
        {{ 80, 200, 255}},
        {{120, 255, 120}},
        {{255, 200,  80}},
        {{200, 120, 255}},
        {{255, 120, 200}},
        {{120, 200, 120}},
        {{200, 200,  80}},
        {{ 80, 120, 255}},
        {{255, 150,  60}},
        {{ 60, 220, 180}},
        {{180,  60, 220}}
    }};


    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    clusters.reserve(clusterIndices.size());
    for (const auto& indices : clusterIndices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(
                new pcl::PointCloud<pcl::PointXYZRGB>());
        cluster->points.reserve(indices.indices.size());
        for (int idx : indices.indices) {
            const auto& pt = clusterCloud->points[static_cast<std::size_t>(idx)];
            cluster->points.push_back(pt);
        }
        cluster->width = static_cast<uint32_t>(cluster->points.size());
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }


    const int n = static_cast<int>(clusters.size());
    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }

    auto findRoot = [&](int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    auto unite = [&](int a, int b) {
        int ra = findRoot(a);
        int rb = findRoot(b);
        if (ra != rb) {
            parent[rb] = ra;
        }
    };

    auto computeBounds = [](pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                            Eigen::Vector3f& minOut,
                            Eigen::Vector3f& maxOut,
                            Eigen::Vector3f& centroidOut) {
        float minX = std::numeric_limits<float>::infinity();
        float minY = std::numeric_limits<float>::infinity();
        float minZ = std::numeric_limits<float>::infinity();
        float maxX = -std::numeric_limits<float>::infinity();
        float maxY = -std::numeric_limits<float>::infinity();
        float maxZ = -std::numeric_limits<float>::infinity();
        double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        std::size_t count = 0;
        for (const auto& pt : cloud->points) {
            if (!pcl::isFinite(pt)) {
                continue;
            }
            minX = std::min(minX, pt.x);
            minY = std::min(minY, pt.y);
            minZ = std::min(minZ, pt.z);
            maxX = std::max(maxX, pt.x);
            maxY = std::max(maxY, pt.y);
            maxZ = std::max(maxZ, pt.z);
            sumX += pt.x;
            sumY += pt.y;
            sumZ += pt.z;
            count++;
        }
        if (count == 0) {
            minOut = Eigen::Vector3f::Zero();
            maxOut = Eigen::Vector3f::Zero();
            centroidOut = Eigen::Vector3f::Zero();
            return;
        }
        minOut = Eigen::Vector3f(minX, minY, minZ);
        maxOut = Eigen::Vector3f(maxX, maxY, maxZ);
        centroidOut = Eigen::Vector3f(
                static_cast<float>(sumX / count),
                static_cast<float>(sumY / count),
                static_cast<float>(sumZ / count));
    };

    std::vector<Eigen::Vector3f> mins(n), maxs(n), cents(n);
    for (int i = 0; i < n; ++i) {
        computeBounds(clusters[i], mins[i], maxs[i], cents[i]);
    }

    if (params.merge_distance_m > 0.0f) {
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                float dist = (cents[i] - cents[j]).norm();
                if (dist <= params.merge_distance_m) {
                    unite(i, j);
                }
            }
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> merged;
    std::vector<int> rootIndex(n, -1);
    for (int i = 0; i < n; ++i) {
        int r = findRoot(i);
        if (rootIndex[r] == -1) {
            rootIndex[r] = static_cast<int>(merged.size());
            merged.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZRGB>()));
        }
        auto& out = merged[rootIndex[r]];
        out->points.insert(out->points.end(), clusters[i]->points.begin(), clusters[i]->points.end());
    }


    result.colored->points.reserve(clusterCloud->size());
    for (std::size_t i = 0; i < merged.size(); ++i) {
        auto& cluster = merged[i];
        if (!cluster || cluster->empty()) {
            continue;
        }
        Eigen::Vector3f minV, maxV, c;
        computeBounds(cluster, minV, maxV, c);
        Eigen::Vector3f size = maxV - minV;
        if (params.min_bbox_size_m > 0.0f) {
            if (size.x() < params.min_bbox_size_m ||
                size.y() < params.min_bbox_size_m ||
                size.z() < params.min_bbox_size_m) {
                continue;
            }
        }
        cluster->width = static_cast<uint32_t>(cluster->points.size());
        cluster->height = 1;
        cluster->is_dense = true;
        result.clusters.push_back(cluster);

        const auto& color = palette[result.clusters.size() % palette.size()];
        for (const auto& pt : cluster->points) {
            pcl::PointXYZRGB coloredPt = pt;
            coloredPt.r = color[0];
            coloredPt.g = color[1];
            coloredPt.b = color[2];
            result.colored->points.push_back(coloredPt);
        }
    }

    result.colored->width = static_cast<uint32_t>(result.colored->points.size());
    result.colored->height = 1;
    result.colored->is_dense = true;
    return result;
}
