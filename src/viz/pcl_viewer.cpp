#include "grasp_system/viz/pcl_viewer.h"
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <cctype>
#include <sstream>
#include <vtkRenderWindow.h>

PclViewer::PclViewer(const std::string& title)
    : viewer_(std::make_shared<pcl::visualization::PCLVisualizer>(title)) {
    viewer_->setBackgroundColor(0.05, 0.05, 0.08);
    viewer_->addCoordinateSystem(0.1, "axes");
    viewer_->initCameraParameters();
    viewer_->setShowFPS(false);
    viewer_->registerKeyboardCallback(&PclViewer::onKeyboardEvent, *this, nullptr);
}

void PclViewer::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    if (!cloud) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr toShow = cloud;
    if (flipXY_) {
        if (!displayCloud_) {
            displayCloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        }
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        tf.matrix()(0, 0) = -1.0f;
        tf.matrix()(1, 1) = -1.0f;
        pcl::transformPointCloud(*cloud, *displayCloud_, tf);
        toShow = displayCloud_;
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(toShow);
    if (!cloudAdded_) {
        viewer_->addPointCloud<pcl::PointXYZRGB>(toShow, rgb, "cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        cloudAdded_ = true;
    } else {
        viewer_->updatePointCloud<pcl::PointXYZRGB>(toShow, rgb, "cloud");
    }
    if (!hasInitialCamera_) {
        viewer_->getCameraParameters(initialCamera_);
        hasInitialCamera_ = true;
    }
}

void PclViewer::updateHud(const std::string& leftText,
                          const std::string& rightText,
                          const std::string& topLeftText) {
    if (!viewer_) {
        return;
    }
    for (int i = 0; i < lastLeftLineCount_; ++i) {
        viewer_->removeShape("hud_left_" + std::to_string(i));
    }
    lastLeftLineCount_ = 0;
    // Place HUD near bottom-left in pixel coords.
    const int lineHeight = 18;
    std::vector<std::string> lines;
    std::string line;
    std::istringstream leftStream(leftText);
    while (std::getline(leftStream, line)) {
        lines.push_back(line);
    }
    for (std::size_t i = 0; i < lines.size(); ++i) {
        int y = 10 + static_cast<int>((lines.size() - 1 - i) * lineHeight);
        viewer_->addText(lines[i], 10, y, 14,
                         0.9, 0.9, 0.9, "hud_left_" + std::to_string(i));
    }
    lastLeftLineCount_ = static_cast<int>(lines.size());
    if (showHelp_) {
        viewer_->removeShape("hud_help");
        int helpY = 10 + lastLeftLineCount_ * lineHeight;
        viewer_->addText("Keys: space - pause | v - reset | z - flip | a - axes | / - help",
                         10, helpY, 16, 0.8, 0.8, 0.8, "hud_help");
    } else {
        viewer_->removeShape("hud_help");
    }

    int winW = 0;
    int winH = 0;
    if (viewer_->getRenderWindow()) {
        const int* size = viewer_->getRenderWindow()->GetSize();
        winW = size ? size[0] : 0;
        winH = size ? size[1] : 0;
    }


    for (int i = 0; i < lastTopLeftLineCount_; ++i) {
        viewer_->removeShape("hud_top_left_" + std::to_string(i));
    }
    lastTopLeftLineCount_ = 0;
    if (!topLeftText.empty()) {
        std::vector<std::string> tlines;
        std::string tline;
        std::istringstream tstream(topLeftText);
        while (std::getline(tstream, tline)) {
            tlines.push_back(tline);
        }
        int startY = (winH > 0) ? (winH - 20) : 10;
        for (std::size_t i = 0; i < tlines.size(); ++i) {
            int y = startY - static_cast<int>(i * lineHeight);
            viewer_->addText(tlines[i], 10, y, 14,
                             0.9, 0.9, 0.9, "hud_top_left_" + std::to_string(i));
        }
        lastTopLeftLineCount_ = static_cast<int>(tlines.size());
    }
    // Top-right info block.
    for (int i = 0; i < lastRightLineCount_; ++i) {
        viewer_->removeShape("hud_right_" + std::to_string(i));
    }
    lastRightLineCount_ = 0;
    if (!rightText.empty()) {
        std::vector<std::string> rlines;
        std::string rline;
        std::istringstream rightStream(rightText);
        while (std::getline(rightStream, rline)) {
            rlines.push_back(rline);
        }
        int rightX = (winW > 0) ? (winW - 220) : 10;
        int startY = (winH > 0) ? (winH - 20) : 10;
        for (std::size_t i = 0; i < rlines.size(); ++i) {
            int y = startY - static_cast<int>(i * lineHeight);
            viewer_->addText(rlines[i], rightX, y, 14,
                             0.9, 0.9, 0.9, "hud_right_" + std::to_string(i));
        }
        lastRightLineCount_ = static_cast<int>(rlines.size());
    }

    // Bottom-right hints (avoid PCL defaults).
    int rightX = (winW > 0) ? (winW - 220) : 10; // lower winW to move more to right
    int bottomY = 10;
    viewer_->removeShape("hud_help_key");
    viewer_->removeShape("hud_help_pcl");
    viewer_->addText("/ - help", rightX, bottomY, 14, 0.85, 0.85, 0.85, "hud_help_key");
    viewer_->addText("h - pcl terminal help", rightX, bottomY + 18, 14,
                     0.8, 0.8, 0.8, "hud_help_pcl");
}

void PclViewer::updateBoundingBoxes(const std::vector<Eigen::Vector3f>& mins,
                                    const std::vector<Eigen::Vector3f>& maxs) {
    if (!viewer_) {
        return;
    }
    for (int i = 0; i < lastBoxCount_; ++i) {
        viewer_->removeShape("bbox_" + std::to_string(i));
    }
    lastBoxCount_ = 0;

    std::size_t count = std::min(mins.size(), maxs.size());
    for (std::size_t i = 0; i < count; ++i) {
        Eigen::Vector3f min = mins[i];
        Eigen::Vector3f max = maxs[i];
        if (flipXY_) {
            float newMinX = -max.x();
            float newMaxX = -min.x();
            float newMinY = -max.y();
            float newMaxY = -min.y();
            min.x() = newMinX;
            max.x() = newMaxX;
            min.y() = newMinY;
            max.y() = newMaxY;
        }
        const std::string id = "bbox_" + std::to_string(i);
        viewer_->addCube(min.x(), max.x(), min.y(), max.y(), min.z(), max.z(),
                         0.9, 0.9, 0.9, id);
        viewer_->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
        viewer_->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, id);
        lastBoxCount_++;
    }
}

void PclViewer::updateBoxLabels(const std::vector<std::string>& labels,
                                const std::vector<Eigen::Vector3f>& positions) {
    if (!viewer_) {
        return;
    }
    static int lastLabelCount = 0;
    for (int i = 0; i < lastLabelCount; ++i) {
        viewer_->removeShape("bbox_label_" + std::to_string(i));
    }
    lastLabelCount = 0;

    std::size_t count = std::min(labels.size(), positions.size());
    for (std::size_t i = 0; i < count; ++i) {
        Eigen::Vector3f pos = positions[i];
        if (flipXY_) {
            pos.x() = -pos.x();
            pos.y() = -pos.y();
        }
        const std::string id = "bbox_label_" + std::to_string(i);
        viewer_->addText3D(labels[i],
                           pcl::PointXYZ(pos.x(), pos.y(), pos.z()),
                           0.008, 1.0, 1.0, 1.0, id);
        lastLabelCount++;
    }
}

void PclViewer::spinOnce(int millis) {
    viewer_->spinOnce(millis);
}

bool PclViewer::isStopped() const {
    return viewer_->wasStopped();
}

bool PclViewer::isPaused() const {
    return paused_;
}

void PclViewer::setFlipXY(bool enabled) {
    flipXY_ = enabled;
}

void PclViewer::toggleAxes() {
    axesShown_ = !axesShown_;
    if (axesShown_) {
        viewer_->addCoordinateSystem(0.1, "axes");
    } else {
        viewer_->removeCoordinateSystem("axes");
    }
}

void PclViewer::setKeyHandler(const std::function<void(char)>& handler) {
    keyHandler_ = handler;
}

void PclViewer::onKeyboardEvent(const pcl::visualization::KeyboardEvent& event, void* /*cookie*/) {
    if (!event.keyDown()) {
        return;
    }
    char key = static_cast<char>(event.getKeyCode());
    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    switch (key) {
        case ' ':
            paused_ = !paused_;
            break;
        case 'v':
            if (hasInitialCamera_) {
                viewer_->setCameraParameters(initialCamera_);
            } else {
                viewer_->resetCamera();
            }
            break;
        case 'z':
            flipXY_ = !flipXY_;
            break;
        case 'a':
            toggleAxes();
            break;
        case '/':
        case '?':
            showHelp_ = !showHelp_;
            break;
        default:
            break;
    }
    if (keyHandler_) {
        keyHandler_(key);
    }
}
