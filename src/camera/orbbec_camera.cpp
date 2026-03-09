

#include "grasp_system/camera/orbbec_camera.h"
#include <libobsensor/hpp/TypeHelper.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace {
bool isDepth16Format(OBFormat format) {
    switch (format) {
        case OB_FORMAT_Y16:
        case OB_FORMAT_Z16:
        case OB_FORMAT_RLE:
        case OB_FORMAT_RVL:
        case OB_FORMAT_Y10:
        case OB_FORMAT_Y11:
        case OB_FORMAT_Y12:
        case OB_FORMAT_Y14:
            return true;
        default:
            return false;
    }
}
} // namespace

OrbbecCamera::OrbbecCamera() = default;

OrbbecCamera::~OrbbecCamera() {
    stop();
}

bool OrbbecCamera::start(int colorWidth, int colorHeight, int depthWidth, int depthHeight, int fps) {
    if (running_) {
        return true;
    }

    try {
        static bool logConfigured = false;
        if (!logConfigured) {
            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
            logConfigured = true;
        }

        pipeline_ = std::make_shared<ob::Pipeline>();
        config_ = std::make_shared<ob::Config>();

        // Just letting the SDK pick a compatible profile for the requested sizes (software align).
        config_->enableVideoStream(OB_SENSOR_COLOR, colorWidth, colorHeight, fps, OB_FORMAT_ANY);
        config_->enableVideoStream(OB_SENSOR_DEPTH, depthWidth, depthHeight, fps, OB_FORMAT_ANY);
        pipeline_->enableFrameSync();
        pipeline_->start(config_);

        alignFilter_ = std::make_shared<ob::Align>(OB_STREAM_DEPTH);
        alignFilter_->setMatchTargetResolution(true);

        running_ = true;
        return true;
    } catch (const ob::Error &e) {
        std::cerr << "Orbbec error: " << e.what() << std::endl;
        return false;
    }
}

void OrbbecCamera::stop() {
    if (pipeline_) {
        try {
            pipeline_->stop();
        } catch (...) {
            // Ignore stop errors during shutdown.
        }
    }
    running_ = false;
}

bool OrbbecCamera::isRunning() const {
    return running_;
}

bool OrbbecCamera::grabFrames(FrameData& out, int timeoutMs) {
    if (!running_ || !pipeline_) {
        return false;
    }

    std::shared_ptr<ob::FrameSet> frameset;
    std::shared_ptr<ob::Frame> colorFrame;
    std::shared_ptr<ob::Frame> depthFrame;

    for (int i = 0; i < 10; ++i) {
        auto rawFrameset = pipeline_->waitForFrameset(timeoutMs);
        if (!rawFrameset) {
            continue;
        }
        auto rawColor = rawFrameset->colorFrame();
        auto rawDepth = rawFrameset->depthFrame();
        if (!rawColor || !rawDepth) {
            continue;
        }
        frameset = rawFrameset;
        if (alignFilter_) {
            auto aligned = alignFilter_->process(rawFrameset);
            if (aligned) {
                frameset = aligned->as<ob::FrameSet>();
            }
        }
        colorFrame = frameset->colorFrame();
        depthFrame = frameset->depthFrame();
        if (colorFrame && depthFrame) {
            break;
        }
    }

    if (!frameset) {
        std::cerr << "No frameset received." << std::endl;
        return false;
    }
    if (!colorFrame || !depthFrame) {
        return false;
    }

    auto colorVideo = colorFrame->as<ob::VideoFrame>();
    auto depthVideo = depthFrame->as<ob::VideoFrame>();
    auto depthFrameTyped = depthFrame->as<ob::DepthFrame>();

    const auto colorFormat = colorVideo->getFormat();
    const auto depthFormat = depthVideo->getFormat();

    if (colorIntrinsics_.width == 0) {
        auto colorIntr = colorVideo->getStreamProfile()->as<ob::VideoStreamProfile>()->getIntrinsic();
        colorIntrinsics_.fx = colorIntr.fx;
        colorIntrinsics_.fy = colorIntr.fy;
        colorIntrinsics_.cx = colorIntr.cx;
        colorIntrinsics_.cy = colorIntr.cy;
        colorIntrinsics_.width = (int)colorIntr.width;
        colorIntrinsics_.height = (int)colorIntr.height;
        colorIntrinsics_.format = ob::TypeHelper::convertOBFormatTypeToString(colorFormat);
    }

    if (depthIntrinsics_.width == 0) {
        auto depthIntr = depthVideo->getStreamProfile()->as<ob::VideoStreamProfile>()->getIntrinsic();
        depthIntrinsics_.fx = depthIntr.fx;
        depthIntrinsics_.fy = depthIntr.fy;
        depthIntrinsics_.cx = depthIntr.cx;
        depthIntrinsics_.cy = depthIntr.cy;
        depthIntrinsics_.width = (int)depthIntr.width;
        depthIntrinsics_.height = (int)depthIntr.height;
        depthIntrinsics_.format = ob::TypeHelper::convertOBFormatTypeToString(depthFormat);
    }

    if (!isDepth16Format(depthFormat)) {
        std::cerr << "Unsupported depth format: "
                  << ob::TypeHelper::convertOBFormatTypeToString(depthFormat) << std::endl;
        return false;
    }

    if (colorFormat == OB_FORMAT_RGB || colorFormat == OB_FORMAT_BGR) {
        cv::Mat colorMat(colorVideo->getHeight(), colorVideo->getWidth(), CV_8UC3,
                         (void*)colorVideo->getData());
        if (colorFormat == OB_FORMAT_RGB) {
            cv::cvtColor(colorMat, out.colorBgr, cv::COLOR_RGB2BGR);
        } else {
            out.colorBgr = colorMat.clone();
        }
    } else if (colorFormat == OB_FORMAT_MJPG) {
        const auto dataSize = colorFrame->getDataSize();
        cv::Mat mjpg(1, (int)dataSize, CV_8UC1, (void*)colorFrame->getData());
        out.colorBgr = cv::imdecode(mjpg, cv::IMREAD_COLOR);
        if (out.colorBgr.empty()) {
            std::cerr << "Failed to decode MJPG color frame." << std::endl;
            return false;
        }
    } else {
        std::cerr << "Unsupported color format: "
                  << ob::TypeHelper::convertOBFormatTypeToString(colorFormat) << std::endl;
        return false;
    }

    cv::Mat depthMat(depthVideo->getHeight(), depthVideo->getWidth(), CV_16UC1,
                     (void*)depthVideo->getData());
    out.depth16 = depthMat.clone();

    out.colorTimestamp = colorFrame->getTimeStampUs();
    out.depthTimestamp = depthFrame->getTimeStampUs();
    out.depthScale = depthFrameTyped->getValueScale();

    return true;
}

OrbbecCamera::Intrinsics OrbbecCamera::getColorIntrinsics() const {
    return colorIntrinsics_;
}

OrbbecCamera::Intrinsics OrbbecCamera::getDepthIntrinsics() const {
    return depthIntrinsics_;
}
