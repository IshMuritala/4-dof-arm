

#include "grasp_system/perception/pointcloud_builder.h"
#include "grasp_system/camera/camera_config.h"
#include <pcl/common/point_tests.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <limits>
#include <iostream>
#include <vector>

// NOTE: All preprocessing disabled in camera_config.h

pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessPointCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input) {
    if (!input) {
        return nullptr;
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noNaN(
            new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*input, *noNaN, idx);
    if (noNaN->empty()) {
        std::cerr << "preprocessPointCloud: no valid points (check DEPTH_*_MM)"
                  << std::endl;
        return noNaN;
    }

    // Commented out - Not using any preprocessing
    // One-time debug: print XYZ bounds before crop to help tune workspace.
    /*
    static bool printedBounds = false;
    if (!printedBounds) {
        float minX = std::numeric_limits<float>::infinity();
        float minY = std::numeric_limits<float>::infinity();
        float minZ = std::numeric_limits<float>::infinity();
        float maxX = -std::numeric_limits<float>::infinity();
        float maxY = -std::numeric_limits<float>::infinity();
        float maxZ = -std::numeric_limits<float>::infinity();
        for (const auto& pt : noNaN->points) {
            if (!pcl::isFinite(pt)) {
                continue;
            }
            if (pt.x < minX) minX = pt.x;
            if (pt.y < minY) minY = pt.y;
            if (pt.z < minZ) minZ = pt.z;
            if (pt.x > maxX) maxX = pt.x;
            if (pt.y > maxY) maxY = pt.y;
            if (pt.z > maxZ) maxZ = pt.z;
        }
        std::cerr << "preprocessPointCloud: bounds (m) "
                  << "x[" << minX << "," << maxX << "] "
                  << "y[" << minY << "," << maxY << "] "
                  << "z[" << minZ << "," << maxZ << "]" << std::endl;
        std::cerr << "preprocessPointCloud: workspace (m) "
                  << "x[" << WORKSPACE_MIN_X_M << "," << WORKSPACE_MAX_X_M << "] "
                  << "y[" << WORKSPACE_MIN_Y_M << "," << WORKSPACE_MAX_Y_M << "] "
                  << "z[" << WORKSPACE_MIN_Z_M << "," << WORKSPACE_MAX_Z_M << "]"
                  << std::endl;
        printedBounds = true;
    }
    */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped = noNaN;
    if (ENABLE_WORKSPACE_CROP) {
        // Manual crop to avoid CropBox issues on some builds
        cropped.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        cropped->points.reserve(noNaN->points.size());
        for (const auto& pt : noNaN->points) {
            if (pt.x < WORKSPACE_MIN_X_M || pt.x > WORKSPACE_MAX_X_M) {
                continue;
            }
            if (pt.y < WORKSPACE_MIN_Y_M || pt.y > WORKSPACE_MAX_Y_M) {
                continue;
            }
            if (pt.z < WORKSPACE_MIN_Z_M || pt.z > WORKSPACE_MAX_Z_M) {
                continue;
            }
            cropped->points.push_back(pt);
        }
        cropped->width = static_cast<uint32_t>(cropped->points.size());
        cropped->height = 1;
        cropped->is_dense = true;
        if (cropped->empty()) {
            std::cerr << "preprocessPointCloud: crop box removed all points; "
                      << "check WORKSPACE_* in camera_config.h" << std::endl;
            return noNaN;
        }
    }

    if (!ENABLE_VOXEL_DOWNSAMPLE || VOXEL_LEAF_SIZE_M <= 0.0f) {
        if (!ENABLE_OUTLIER_REMOVAL) {
            return cropped;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr down = cropped;
    if (ENABLE_VOXEL_DOWNSAMPLE && VOXEL_LEAF_SIZE_M > 0.0f) {
        // Voxel downsample for speed.
        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        vox.setLeafSize(VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M, VOXEL_LEAF_SIZE_M);
        vox.setInputCloud(cropped);
        down.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        vox.filter(*down);
        if (down->empty()) {
            std::cerr << "preprocessPointCloud: voxel removed all points; "
                      << "returning pre-voxel cloud" << std::endl;
            return cropped;
        }
    }

    if (!ENABLE_OUTLIER_REMOVAL) {
        return down;
    }

    if (down->size() < static_cast<size_t>(OUTLIER_MEAN_K)) {
        return down;
    }

    // Statistical outlier removal to clean sparse noise.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(down);
    sor.setMeanK(OUTLIER_MEAN_K);
    sor.setStddevMulThresh(OUTLIER_STDDEV_MUL);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZRGB>());
    sor.filter(*filtered);
    if (filtered->empty()) {
        std::cerr << "preprocessPointCloud: SOR removed all points; "
                  << "returning pre-SOR cloud" << std::endl;
        return down;
    }
    return filtered;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildPointCloud(
        const cv::Mat& depth16,
        const cv::Mat& colorBgr,
        const OrbbecCamera::Intrinsics& depthIntr,
        float depthScale) {

    if (depth16.empty()) {
        std::cerr << "buildPointCloud: depth16 is empty" << std::endl;
        return nullptr;
    }

    if (!colorBgr.empty() &&
        (colorBgr.rows != depth16.rows || colorBgr.cols != depth16.cols)) {
        std::cerr << "buildPointCloud: color/depth size mismatch" << std::endl;
        return nullptr;
    }

    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud->width = static_cast<uint32_t>(depth16.cols);
    cloud->height = static_cast<uint32_t>(depth16.rows);
    cloud->is_dense = false;
    cloud->points.resize(static_cast<size_t>(cloud->width) * cloud->height);

    const float fx = depthIntr.fx;
    const float fy = depthIntr.fy;
    const float cx = depthIntr.cx;
    const float cy = depthIntr.cy;

    const float minMm = static_cast<float>(DEPTH_MIN_MM);
    const float maxMm = static_cast<float>(DEPTH_MAX_MM);

    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (int v = 0; v < depth16.rows; ++v) {
        const uint16_t* depthRow = depth16.ptr<uint16_t>(v);
        const cv::Vec3b* colorRow = colorBgr.empty() ? nullptr : colorBgr.ptr<cv::Vec3b>(v);

        for (int u = 0; u < depth16.cols; ++u) {
            const size_t idx = static_cast<size_t>(v) * depth16.cols + u;
            auto& pt = cloud->points[idx];

            uint16_t raw = depthRow[u];
            if (raw == 0) {
                pt.x = nan; pt.y = nan; pt.z = nan;
                pt.r = pt.g = pt.b = 0;
                continue;
            }


            float depthMm = static_cast<float>(raw) * depthScale;
            if (depthMm < minMm || depthMm > maxMm) {
                pt.x = nan; pt.y = nan; pt.z = nan;
                pt.r = pt.g = pt.b = 0;
                continue;
            }

            // Convert to meters for PCL
            float z = depthMm / 1000.0f;
            float x = (static_cast<float>(u) - cx) * z / fx;
            float y = (static_cast<float>(v) - cy) * z / fy;

            pt.x = x;
            pt.y = y;
            pt.z = z;

            if (colorRow) {
                const cv::Vec3b& bgr = colorRow[u];
                pt.b = bgr[0];
                pt.g = bgr[1];
                pt.r = bgr[2];
            } else {
                pt.r = pt.g = pt.b = 0;
            }
        }
    }

    return preprocessPointCloud(cloud);
}
