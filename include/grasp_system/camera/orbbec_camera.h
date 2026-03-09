#ifndef INC_4DOF_ARM_ORBBEC_CAMERA_H
#define INC_4DOF_ARM_ORBBEC_CAMERA_H

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Filter.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <memory>
#include <cstdint>

class OrbbecCamera {
public:
    struct Intrinsics {
        float fx = 0.0f;
        float fy = 0.0f;
        float cx = 0.0f;
        float cy = 0.0f;
        int width = 0;
        int height = 0;
        std::string format;
    };

    struct FrameData {
        cv::Mat colorBgr;
        cv::Mat depth16;
        uint64_t colorTimestamp = 0;
        uint64_t depthTimestamp = 0;
        float depthScale = 1.0f; // raw depth * scale = millimeters
    };

    OrbbecCamera();
    ~OrbbecCamera();

    bool start(int colorWidth = 1920, int colorHeight = 1080,
               int depthWidth = 1280, int depthHeight = 800,
               int fps = 30);
    void stop();
    bool isRunning() const;

    // Grabs one aligned color+depth frameset (color is aligned to depth)
    bool grabFrames(FrameData& out, int timeoutMs = 1000);

    Intrinsics getColorIntrinsics() const;
    Intrinsics getDepthIntrinsics() const;

private:
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::shared_ptr<ob::Config> config_;
    std::shared_ptr<ob::VideoStreamProfile> colorProfile_;
    std::shared_ptr<ob::VideoStreamProfile> depthProfile_;
    std::shared_ptr<ob::Align> alignFilter_;
    Intrinsics colorIntrinsics_;
    Intrinsics depthIntrinsics_;
    bool running_ = false;
};

#endif //INC_4DOF_ARM_ORBBEC_CAMERA_H
