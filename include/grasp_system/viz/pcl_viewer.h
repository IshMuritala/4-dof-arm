#ifndef INC_4DOF_ARM_PCL_VIEWER_H
#define INC_4DOF_ARM_PCL_VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

class PclViewer {
public:
    explicit PclViewer(const std::string& title = "PCL Viewer");

    void updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void updateHud(const std::string& leftText,
                   const std::string& rightText = std::string(),
                   const std::string& topLeftText = std::string());
    void updateBoundingBoxes(const std::vector<Eigen::Vector3f>& mins,
                             const std::vector<Eigen::Vector3f>& maxs);
    void updateBoxLabels(const std::vector<std::string>& labels,
                         const std::vector<Eigen::Vector3f>& positions);
    void spinOnce(int millis = 10);
    bool isStopped() const;
    bool isPaused() const;
    // Viewer-only flip
    void setFlipXY(bool enabled);
    void toggleAxes();
    void setKeyHandler(const std::function<void(char)>& handler);

private:
    void onKeyboardEvent(const pcl::visualization::KeyboardEvent& event, void* cookie);

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool cloudAdded_ = false;
    bool flipXY_ = true;
    bool axesShown_ = true;
    bool paused_ = false;
    bool showHelp_ = false;
    bool hasInitialCamera_ = false;
    pcl::visualization::Camera initialCamera_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud_;
    std::function<void(char)> keyHandler_;
    int lastLeftLineCount_ = 0;
    int lastRightLineCount_ = 0;
    int lastTopLeftLineCount_ = 0;
    int lastBoxCount_ = 0;
};

#endif //INC_4DOF_ARM_PCL_VIEWER_H
