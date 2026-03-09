#ifndef INC_4DOF_ARM_POINTCLOUD_BUILDER_H
#define INC_4DOF_ARM_POINTCLOUD_BUILDER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include "grasp_system/camera/orbbec_camera.h"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildPointCloud(
        const cv::Mat& depth16,
        const cv::Mat& colorBgr,
        const OrbbecCamera::Intrinsics& depthIntr,
        float depthScale);

// Removes NaNs, crops to workspace bounds, then voxel-downsamples.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

#endif //INC_4DOF_ARM_POINTCLOUD_BUILDER_H
