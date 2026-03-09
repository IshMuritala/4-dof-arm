#ifndef INC_4DOF_ARM_SEGMENTATION_H
#define INC_4DOF_ARM_SEGMENTATION_H

#include <cstddef>
#include <string>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>


struct PlaneSegmentationParams {
    float distance_threshold_m = 0.008f;
    int max_iterations = 200;
    bool use_axis_constraint = false;
    Eigen::Vector3f axis = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    float eps_angle_deg = 15.0f;
    std::size_t min_inliers = 500;
    float min_inlier_ratio = 0.05f;
    bool verbose = false;
};

struct PlaneSegmentationResult {
    bool success = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
};


struct ClusterParams {
    float tolerance_m = 0.02f;
    int min_cluster_size = 100;
    int max_cluster_size = 25000;
    float voxel_leaf_m = 0.004f;
    float merge_distance_m = 0.05f;
    float min_bbox_size_m = 0.0f;
    bool verbose = false;
};

struct ClusterResult {
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored;
};


PlaneSegmentationParams loadPlaneSegmentationParams(const std::string& yamlPath);


ClusterParams loadClusterParams(const std::string& yamlPath);


PlaneSegmentationResult segmentTablePlane(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
        const PlaneSegmentationParams& params = PlaneSegmentationParams());


ClusterResult clusterObjects(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
        const ClusterParams& params = ClusterParams());

#endif //INC_4DOF_ARM_SEGMENTATION_H
