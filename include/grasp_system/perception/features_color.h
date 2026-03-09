#ifndef INC_4DOF_ARM_FEATURES_COLOR_H
#define INC_4DOF_ARM_FEATURES_COLOR_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


struct ClusterFeatures {
    bool valid = false;
    std::size_t point_count = 0;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_size = Eigen::Vector3f::Zero();
    Eigen::Vector3f median_rgb = Eigen::Vector3f::Zero(); // 0-255
    float aspect_xy = 0.0f;
    float aspect_xz = 0.0f;
    float aspect_yz = 0.0f;
    float confidence = 0.0f;
};


ClusterFeatures extractClusterFeatures(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cluster,
        int min_points = 100,
        int max_points = 25000);


std::vector<ClusterFeatures> extractClusterFeatures(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clusters,
        int min_points = 100,
        int max_points = 25000);

#endif //INC_4DOF_ARM_FEATURES_COLOR_H
