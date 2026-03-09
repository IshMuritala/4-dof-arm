//
// Camera-to-robot calibration helper
// - Uses OpenCV window + mouse click to pick a 3D camera point
// - Uses manual servo control keys (same as manual_control)
// - Captures (P_cam, P_robot) pairs and solves rigid transform
// - Writes config/calibration.yaml
//

#include "grasp_system/camera/orbbec_camera.h"
#include "grasp_system/comms/serial_arduino.h"
#include "grasp_system/planning/fk.h"
#include "grasp_system/planning/robot_config.h"
#include "grasp_system/camera/camera_config.h"
#include "protocol/protocol.h"

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>

namespace {

constexpr int SERVO_COUNT = 5;
constexpr int SERVO_GRIPPER = 4;
constexpr int GRIPPER_STEP = 25;
constexpr int GRIPPER_HOME = 2800;  // keep in sync with servo_limits.h !!
constexpr int JOINT_DONE_TIMEOUT_MS = 7000;
constexpr int GRIPPER_DONE_TIMEOUT_MS = 10000;
constexpr int EEPROM_READ_ATTEMPTS = 3;

constexpr int kAckTimeoutMs = 1000;
constexpr int kDoneTimeoutMs = 12000;
constexpr int kReadyBase = 90;
constexpr int kReadyShoulder = 145;
constexpr int kReadyElbow = 145;
constexpr int kReadyWrist = 80;
constexpr int kDisplayWidth = 1280;
constexpr int kDisplayHeight = 720;

const int servoMin[SERVO_COUNT] = {0, 0, 0, 48, 1000};
const int servoMax[SERVO_COUNT] = {180, 180, 150, 115, 4000};

const char* servoNames[] = {"Base", "Shoulder", "Elbow", "Wrist", "Gripper"};

// Raw input helpers (same as manual_control)
void enableRawMode() {
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    term.c_iflag &= ~(IXON | ICRNL | INLCR);
    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);
    setbuf(stdin, NULL);
}

void disableRawMode() {
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    fcntl(STDIN_FILENO, F_SETFL, 0);
}

bool isGripper(int servoIndex) {
    return servoIndex == SERVO_GRIPPER;
}

int stepForServo(int servoIndex) {
    return isGripper(servoIndex) ? GRIPPER_STEP : 1;
}

int doneTimeoutForServo(int servoIndex) {
    return isGripper(servoIndex) ? GRIPPER_DONE_TIMEOUT_MS : JOINT_DONE_TIMEOUT_MS;
}

bool sendAdjustServo(SerialArduino& arduino, uint8_t channel, int8_t increment) {
    AdjustServoData adjustData;
    adjustData.channel = channel;
    adjustData.increment = increment;
    return arduino.sendCommand(CMD_ADJUST_SERVO, (uint8_t*)&adjustData, sizeof(adjustData));
}

bool sendHome(SerialArduino& arduino) {
    return arduino.sendCommand(CMD_HOME, nullptr, 0);
}

bool sendEmergencyStop(SerialArduino& arduino) {
    return arduino.sendEmergencyStop();
}

uint16_t clampU16(int value, int min_v, int max_v) {
    if (value < min_v) return static_cast<uint16_t>(min_v);
    if (value > max_v) return static_cast<uint16_t>(max_v);
    return static_cast<uint16_t>(value);
}

bool sendMoveAndWait(SerialArduino& arduino,
                     const RobotConfig& cfg,
                     const JointAnglesDeg& joints,
                     int gripper,
                     const std::string& label) {
    uint16_t base = clampU16(static_cast<int>(std::lround(joints.base)),
                             static_cast<int>(cfg.joint_limits.base_min),
                             static_cast<int>(cfg.joint_limits.base_max));
    uint16_t shoulder = clampU16(static_cast<int>(std::lround(joints.shoulder)),
                                 static_cast<int>(cfg.joint_limits.shoulder_min),
                                 static_cast<int>(cfg.joint_limits.shoulder_max));
    uint16_t elbow = clampU16(static_cast<int>(std::lround(joints.elbow)),
                              static_cast<int>(cfg.joint_limits.elbow_min),
                              static_cast<int>(cfg.joint_limits.elbow_max));
    uint16_t wrist = clampU16(static_cast<int>(std::lround(joints.wrist)),
                              static_cast<int>(cfg.joint_limits.wrist_min),
                              static_cast<int>(cfg.joint_limits.wrist_max));
    uint16_t gripper_u = clampU16(gripper,
                                  static_cast<int>(cfg.gripper.open),
                                  static_cast<int>(cfg.gripper.closed));

    std::cout << "Step: " << label << std::endl;
    if (!arduino.sendMoveJoints(base, shoulder, elbow, wrist, gripper_u)) {
        std::cerr << "Failed to send move command for step: " << label << std::endl;
        return false;
    }
    if (!arduino.waitForAckAndDone(kAckTimeoutMs, kDoneTimeoutMs)) {
        std::cerr << "Timeout waiting for ACK/DONE on step: " << label << std::endl;
        return false;
    }
    return true;
}

bool readEepromAngles(SerialArduino& arduino, JointAnglesDeg& joints, int& gripper) {
    uint8_t responseType = 0;
    uint8_t responseData[MAX_MESSAGE_LENGTH] = {0};
    uint8_t responseLen = 0;

    if (!arduino.sendGetAngles()) {
        return false;
    }
    if (!arduino.waitForResponse(responseType, responseData, responseLen, 1000)) {
        return false;
    }
    if (responseType != RESP_ACK || responseLen != sizeof(GetAnglesData)) {
        return false;
    }
    const GetAnglesData* angles = reinterpret_cast<const GetAnglesData*>(responseData);
    joints.base = static_cast<float>(angles->base);
    joints.shoulder = static_cast<float>(angles->shoulder);
    joints.elbow = static_cast<float>(angles->elbow);
    joints.wrist = static_cast<float>(angles->wrist);
    gripper = static_cast<int>(angles->gripper);
    return true;
}

bool waitForArduinoReady(SerialArduino& arduino, int totalTimeoutMs) {
    using namespace std::chrono;
    const auto start = steady_clock::now();
    while (duration_cast<milliseconds>(steady_clock::now() - start).count() < totalTimeoutMs) {
        if (!arduino.sendPing()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        uint8_t responseType = 0;
        uint8_t responseData[MAX_MESSAGE_LENGTH] = {0};
        uint8_t responseLen = 0;
        if (arduino.waitForResponse(responseType, responseData, responseLen, 500)) {
            if (responseType == RESP_ACK) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

std::filesystem::path findProjectRoot() {
    std::filesystem::path dir = std::filesystem::current_path();
    for (int i = 0; i < 6; ++i) {
        if (std::filesystem::exists(dir / "config" / "robot.yaml")) {
            return dir;
        }
        if (!dir.has_parent_path()) {
            break;
        }
        dir = dir.parent_path();
    }
    return {};
}

bool sampleDepthMedian(const cv::Mat& depth16,
                       int u, int v,
                       float depthScale,
                       float& depthMmOut) {
    if (depth16.empty() || depth16.type() != CV_16UC1) {
        return false;
    }
    const int radius = 2; // 5x5 window
    std::vector<float> depths;
    depths.reserve(25);

    const int minU = std::max(0, u - radius);
    const int maxU = std::min(depth16.cols - 1, u + radius);
    const int minV = std::max(0, v - radius);
    const int maxV = std::min(depth16.rows - 1, v + radius);

    for (int y = minV; y <= maxV; ++y) {
        const uint16_t* row = depth16.ptr<uint16_t>(y);
        for (int x = minU; x <= maxU; ++x) {
            uint16_t raw = row[x];
            if (raw == 0) {
                continue;
            }
            float depthMm = static_cast<float>(raw) * depthScale;
            if (depthMm < static_cast<float>(DEPTH_MIN_MM) ||
                depthMm > static_cast<float>(DEPTH_MAX_MM)) {
                continue;
            }
            depths.push_back(depthMm);
        }
    }

    if (depths.empty()) {
        return false;
    }

    const size_t mid = depths.size() / 2;
    std::nth_element(depths.begin(), depths.begin() + mid, depths.end());
    depthMmOut = depths[mid];
    return true;
}

bool pixelToCameraPoint(const OrbbecCamera::Intrinsics& intr,
                        const cv::Mat& depth16,
                        float depthScale,
                        int u, int v,
                        Eigen::Vector3f& out) {
    float depthMm = 0.0f;
    if (!sampleDepthMedian(depth16, u, v, depthScale, depthMm)) {
        return false;
    }
    const float z = depthMm / 1000.0f;
    const float x = (static_cast<float>(u) - intr.cx) * z / intr.fx;
    const float y = (static_cast<float>(v) - intr.cy) * z / intr.fy;
    out = Eigen::Vector3f(x, y, z);
    return true;
}

bool solveRigidTransform(const std::vector<Eigen::Vector3f>& cam,
                         const std::vector<Eigen::Vector3f>& robot,
                         Eigen::Matrix3f& R,
                         Eigen::Vector3f& t,
                         float& rmsOut,
                         float& maxOut) {
    if (cam.size() != robot.size() || cam.size() < 3) {
        return false;
    }

    Eigen::Vector3f c_cam = Eigen::Vector3f::Zero();
    Eigen::Vector3f c_robot = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < cam.size(); ++i) {
        c_cam += cam[i];
        c_robot += robot[i];
    }
    c_cam /= static_cast<float>(cam.size());
    c_robot /= static_cast<float>(robot.size());

    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < cam.size(); ++i) {
        Eigen::Vector3f pc = cam[i] - c_cam;
        Eigen::Vector3f pr = robot[i] - c_robot;
        H += pc * pr.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    R = V * U.transpose();

    if (R.determinant() < 0.0f) {
        V.col(2) *= -1.0f;
        R = V * U.transpose();
    }

    t = c_robot - R * c_cam;

    float sumSq = 0.0f;
    float maxErr = 0.0f;
    for (size_t i = 0; i < cam.size(); ++i) {
        Eigen::Vector3f pred = R * cam[i] + t;
        float err = (pred - robot[i]).norm();
        sumSq += err * err;
        if (err > maxErr) {
            maxErr = err;
        }
    }
    rmsOut = std::sqrt(sumSq / static_cast<float>(cam.size()));
    maxOut = maxErr;
    return true;
}

bool writeCalibrationYaml(const std::filesystem::path& path,
                          const Eigen::Matrix3f& R,
                          const Eigen::Vector3f& t,
                          float rmsError,
                          size_t numPoints) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }

    out.setf(std::ios::fixed);
    out.precision(6);

    out << "calibration:\n";
    out << "  T_robot_from_cam:\n";
    out << "    - [" << R(0,0) << ", " << R(0,1) << ", " << R(0,2) << ", " << t.x() << "]\n";
    out << "    - [" << R(1,0) << ", " << R(1,1) << ", " << R(1,2) << ", " << t.y() << "]\n";
    out << "    - [" << R(2,0) << ", " << R(2,1) << ", " << R(2,2) << ", " << t.z() << "]\n";
    out << "    - [0, 0, 0, 1]\n";
    out << "  rms_error_m: " << rmsError << "\n";
    out << "  num_points: " << numPoints << "\n";
    out << "  notes: \"Calibration using pin + jaw-center TCP\"\n";

    return true;
}

struct ClickState {
    bool hasClick = false;
    cv::Point lastPx; // display coordinates
    float scaleX = 1.0f; // display -> source scale
    float scaleY = 1.0f;
};

void onMouse(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    auto* state = reinterpret_cast<ClickState*>(userdata);
    if (!state) {
        return;
    }
    state->hasClick = true;
    state->lastPx = cv::Point(x, y);
}

} // namespace

int main(int argc, char* argv[]) {
    std::string port;
    if (argc > 1 && argv[1]) {
        port = argv[1];
    }

    std::filesystem::path root = findProjectRoot();
    if (root.empty()) {
        std::cerr << "Could not locate project root (missing config/robot.yaml). "
                     "Run from the repo root or cmake-build-debug." << std::endl;
        return 1;
    }

    const std::string robotConfigPath = (root / "config" / "robot.yaml").string();
    RobotConfig cfg = loadRobotConfig(robotConfigPath);
    if (!cfg.valid) {
        std::cerr << "Failed to load robot config: " << robotConfigPath << std::endl;
        return 1;
    }

    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
        std::cerr << "Warning: stdin/stdout is not a TTY. Real-time key input"
                     " requires a terminal. Enable CLion \"Emulate terminal\""
                     " or run in a system terminal." << std::endl;
    }

    SerialArduino arduino;
    std::cout << "Connecting to Arduino..." << std::endl;
    bool connected = false;
    if (!port.empty()) {
        connected = arduino.connect(port, SERIAL_BAUD_RATE);
    } else {
        connected = arduino.connectAuto(SERIAL_BAUD_RATE);
    }
    if (!connected) {
        std::cerr << "Failed to connect to Arduino." << std::endl;
        return 1;
    }

    // Allow Arduino to boot and emit startup text, then flush.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    arduino.flushInput();

    if (!waitForArduinoReady(arduino, 12000)) {
        std::cerr << "Warning: Arduino did not respond to PING within timeout. Continuing." << std::endl;
    }

    JointAnglesDeg eepromJoints;
    int eepromGripper = cfg.gripper.open;
    bool haveEeprom = false;
    for (int attempt = 0; attempt < EEPROM_READ_ATTEMPTS && !haveEeprom; ++attempt) {
        haveEeprom = readEepromAngles(arduino, eepromJoints, eepromGripper);
        if (!haveEeprom) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            arduino.flushInput();
        }
    }
    if (haveEeprom) {
        std::cout << "EEPROM angles: [base=" << eepromJoints.base
                  << ", shoulder=" << eepromJoints.shoulder
                  << ", elbow=" << eepromJoints.elbow
                  << ", wrist=" << eepromJoints.wrist
                  << ", gripper=" << eepromGripper << "]" << std::endl;
    } else {
        std::cout << "EEPROM angles read failed (continuing)." << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    JointAnglesDeg readyJoints;
    readyJoints.base = static_cast<float>(kReadyBase);
    readyJoints.shoulder = static_cast<float>(kReadyShoulder);
    readyJoints.elbow = static_cast<float>(kReadyElbow);
    readyJoints.wrist = static_cast<float>(kReadyWrist);
    if (!sendMoveAndWait(arduino, cfg, readyJoints, cfg.gripper.open, "ready")) {
        return 1;
    }

    OrbbecCamera cam;
    if (!cam.start()) {
        std::cerr << "Failed to start Orbbec camera" << std::endl;
        return 1;
    }

    cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
    cv::resizeWindow("Calibration", kDisplayWidth, kDisplayHeight);
    ClickState clickState;
    cv::setMouseCallback("Calibration", onMouse, &clickState);

    // Manual control state (mirrors Arduino tracking)
    int currentAngles[SERVO_COUNT] = {90, 90, 90, 90, GRIPPER_HOME};
    int selectedServo = 0;

    // Try to seed currentAngles with EEPROM readings.
    if (haveEeprom) {
        currentAngles[0] = static_cast<int>(std::lround(eepromJoints.base));
        currentAngles[1] = static_cast<int>(std::lround(eepromJoints.shoulder));
        currentAngles[2] = static_cast<int>(std::lround(eepromJoints.elbow));
        currentAngles[3] = static_cast<int>(std::lround(eepromJoints.wrist));
        currentAngles[4] = eepromGripper;
    }

    std::string angleInput;
    bool angleEntryActive = false;

    bool autoStepActive = false;
    int autoStepDir = 0; // +1 or -1
    std::chrono::steady_clock::time_point nextStepTime;
    const auto autoStepInterval = std::chrono::milliseconds(120);

    std::vector<Eigen::Vector3f> camPoints;
    std::vector<Eigen::Vector3f> robotPoints;

    bool solved = false;
    Eigen::Matrix3f solvedR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f solvedT = Eigen::Vector3f::Zero();
    float solvedRms = 0.0f;
    float solvedMax = 0.0f;

    bool lastClickDepthValid = false;
    float lastClickDepthMm = 0.0f;

    bool running = true;
    bool eStopTriggered = false;

    OrbbecCamera::FrameData lastFrame;
    OrbbecCamera::Intrinsics lastDepthIntr;
    bool hasFrame = false;

    enableRawMode();

    while (running) {
        int cvKey = -1;
        // Grab a frame (keep timeout short for responsiveness).
        OrbbecCamera::FrameData frame;
        if (cam.grabFrames(frame, 120)) {
            lastFrame = frame;
            lastDepthIntr = cam.getDepthIntrinsics();
            hasFrame = true;
        }

        if (hasFrame && !lastFrame.colorBgr.empty()) {
            cv::Mat display(kDisplayHeight, kDisplayWidth, CV_8UC3);
            cv::resize(lastFrame.colorBgr, display, cv::Size(kDisplayWidth, kDisplayHeight));
            clickState.scaleX = static_cast<float>(lastFrame.colorBgr.cols) / static_cast<float>(kDisplayWidth);
            clickState.scaleY = static_cast<float>(lastFrame.colorBgr.rows) / static_cast<float>(kDisplayHeight);

            // Draw last click marker.
            if (clickState.hasClick) {
                cv::circle(display, clickState.lastPx, 5, cv::Scalar(0, 255, 255), 2);
                int srcU = static_cast<int>(std::lround(clickState.lastPx.x * clickState.scaleX));
                int srcV = static_cast<int>(std::lround(clickState.lastPx.y * clickState.scaleY));
                srcU = std::clamp(srcU, 0, lastFrame.depth16.cols - 1);
                srcV = std::clamp(srcV, 0, lastFrame.depth16.rows - 1);
                float depthMm = 0.0f;
                lastClickDepthValid = sampleDepthMedian(lastFrame.depth16,
                                                        srcU,
                                                        srcV,
                                                        lastFrame.depthScale,
                                                        depthMm);
                if (lastClickDepthValid) {
                    lastClickDepthMm = depthMm;
                }
            }

            const int margin = 12;
            int line = 0;

            // Top-left tip + current selection
            cv::putText(display,
                        "Tip: keep wrist angle consistent (e.g., 48 deg) for repeatability",
                        cv::Point(margin, margin + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            std::string selText = std::string("Selected: ") + servoNames[selectedServo] +
                                  " = " + std::to_string(currentAngles[selectedServo]) +
                                  (isGripper(selectedServo) ? " (pos)" : " deg");
            cv::putText(display, selText,
                        cv::Point(margin, margin + 38),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

            // Top-right last click depth
            std::string depthText = lastClickDepthValid
                ? ("Last click depth: " + std::to_string(lastClickDepthMm) + " mm")
                : "Last click depth: invalid";
            int depthBaseline = 0;
            cv::Size depthSize = cv::getTextSize(depthText, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &depthBaseline);
            cv::Point depthPos(display.cols - depthSize.width - margin, margin + 18);
            cv::putText(display, depthText, depthPos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            // Bottom-left key binds
            std::vector<std::string> help = {
                "Keys: 1-5 select | i/p step | I/P auto | s stop auto",
                "g abs angle | h home | e estop",
                "c capture | u undo | r reset | v validate/solve | w write | q quit"
            };

            int baseY = display.rows - margin - 2;
            for (int i = static_cast<int>(help.size()) - 1; i >= 0; --i) {
                cv::putText(display, help[i],
                            cv::Point(margin, baseY - (help.size() - 1 - i) * 18),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }

            // Bottom-right status stack (angles + capture + RMS)
            std::vector<std::string> rightLines;
            rightLines.reserve(4);

            std::string line1 = "Base: " + std::to_string(currentAngles[0]) +
                                "  Shoulder: " + std::to_string(currentAngles[1]) +
                                "  Elbow: " + std::to_string(currentAngles[2]);
            std::string line2 = "Wrist: " + std::to_string(currentAngles[3]) +
                                "  Gripper: " + std::to_string(currentAngles[4]);

            std::string countText = "Captured: " + std::to_string(camPoints.size()) + " (min 6)";
            rightLines.push_back(countText);
            if (solved) {
                rightLines.push_back("RMS: " + std::to_string(solvedRms) + " m");
            }
            rightLines.push_back(line2);
            rightLines.push_back(line1);

            int baseline = 0;
            int y = display.rows - margin;
            for (const auto& lineText : rightLines) {
                cv::Size size = cv::getTextSize(lineText, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
                cv::Point pos(display.cols - size.width - margin, y);
                cv::putText(display, lineText, pos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
                y -= 20;
            }

            cv::imshow("Calibration", display);
            cvKey = cv::waitKey(1);
        }

        // Auto-step
        auto now = std::chrono::steady_clock::now();
        if (autoStepActive && now >= nextStepTime) {
            int step = stepForServo(selectedServo);
            int nextAngle = currentAngles[selectedServo] + (autoStepDir * step);
            if (nextAngle > servoMax[selectedServo] || nextAngle < servoMin[selectedServo]) {
                std::cerr << "\nWarning: min/max reached for "
                          << servoNames[selectedServo] << std::endl;
                autoStepActive = false;
            } else {
                if (sendAdjustServo(arduino, selectedServo, static_cast<int8_t>(autoStepDir * step))) {
                    if (arduino.waitForAckAndDone(500, doneTimeoutForServo(selectedServo))) {
                        currentAngles[selectedServo] = nextAngle;
                    } else {
                        std::cerr << "\nTimeout waiting for auto-step ACK/DONE" << std::endl;
                        autoStepActive = false;
                    }
                } else {
                    std::cerr << "\nFailed to send auto-step command" << std::endl;
                    autoStepActive = false;
                }
            }
            nextStepTime = now + autoStepInterval;
        }

        // Keyboard input (OpenCV window takes priority if focused)
        char c;
        bool haveKey = false;
        if (cvKey > 0 && cvKey < 256) {
            c = static_cast<char>(cvKey);
            haveKey = true;
        } else if (read(STDIN_FILENO, &c, 1) > 0) {
            haveKey = true;
        }
        if (haveKey) {
            // Angle entry mode
            if (angleEntryActive) {
                if (c >= '0' && c <= '9') {
                    size_t maxDigits = isGripper(selectedServo) ? 4 : 3;
                    if (angleInput.size() < maxDigits) {
                        angleInput.push_back(c);
                    }
                    continue;
                }

                if ((c == 127 || c == 8) && !angleInput.empty()) {
                    angleInput.pop_back();
                    continue;
                }

                if (c == '\n' || c == '\r' || c == 'g' || c == 'G') {
                    if (!angleInput.empty()) {
                        int angle = std::stoi(angleInput);
                        int minA = servoMin[selectedServo];
                        int maxA = servoMax[selectedServo];

                        if (angle < minA || angle > maxA) {
                            std::cerr << "\nWarning: angle " << angle
                                      << " is out of range (" << minA
                                      << "-" << maxA << ") for "
                                      << servoNames[selectedServo] << std::endl;
                        } else {
                            if (arduino.sendSetServoAbs(selectedServo, static_cast<uint16_t>(angle))) {
                                if (arduino.waitForAckAndDone(1000, doneTimeoutForServo(selectedServo))) {
                                    currentAngles[selectedServo] = angle;
                                }
                            } else {
                                std::cerr << "\nFailed to send absolute angle command" << std::endl;
                            }
                        }
                    }
                    angleInput.clear();
                    angleEntryActive = false;
                    continue;
                }

                angleInput.clear();
                angleEntryActive = false;
                continue;
            }

            if (c >= '1' && c <= '5') {
                selectedServo = c - '1';
                autoStepActive = false;
            } else if (c == 'g' || c == 'G') {
                angleEntryActive = true;
                angleInput.clear();
                autoStepActive = false;
            } else if (c == 's' || c == 'S') {
                // Stop auto-step only.
                if (autoStepActive) {
                    autoStepActive = false;
                }
            } else if (c == 'I') {
                autoStepActive = true;
                autoStepDir = 1;
                nextStepTime = std::chrono::steady_clock::now();
            } else if (c == 'P') {
                autoStepActive = true;
                autoStepDir = -1;
                nextStepTime = std::chrono::steady_clock::now();
            } else if (c == 'i' || c == 'I') {
                int step = stepForServo(selectedServo);
                int nextAngle = currentAngles[selectedServo] + step;
                if (nextAngle > servoMax[selectedServo]) {
                    std::cerr << "\nWarning: max angle reached for "
                              << servoNames[selectedServo] << std::endl;
                    continue;
                }
                if (sendAdjustServo(arduino, selectedServo, static_cast<int8_t>(step))) {
                    if (arduino.waitForAckAndDone(1000, doneTimeoutForServo(selectedServo))) {
                        currentAngles[selectedServo] += step;
                    }
                } else {
                    std::cerr << "\nFailed to send adjust command" << std::endl;
                }
            } else if (c == 'p' || c == 'P') {
                int step = stepForServo(selectedServo);
                int nextAngle = currentAngles[selectedServo] - step;
                if (nextAngle < servoMin[selectedServo]) {
                    std::cerr << "\nWarning: min angle reached for "
                              << servoNames[selectedServo] << std::endl;
                    continue;
                }
                if (sendAdjustServo(arduino, selectedServo, static_cast<int8_t>(-step))) {
                    if (arduino.waitForAckAndDone(1000, doneTimeoutForServo(selectedServo))) {
                        currentAngles[selectedServo] -= step;
                    }
                } else {
                    std::cerr << "\nFailed to send adjust command" << std::endl;
                }
            } else if (c == 'h' || c == 'H') {
                std::cout << "\nHoming all servos..." << std::endl;
                if (sendHome(arduino)) {
                    if (arduino.waitForAckAndDone(1000, doneTimeoutForServo(SERVO_GRIPPER))) {
                        for (int i = 0; i < SERVO_COUNT; i++) {
                            currentAngles[i] = (i == SERVO_GRIPPER) ? GRIPPER_HOME : 90;
                        }
                        std::cout << "\u2713 Homing complete" << std::endl;
                    }
                } else {
                    std::cerr << "Failed to send home command" << std::endl;
                }
            } else if (c == 'e' || c == 'E') {
                std::cout << "\nSending EMERGENCY STOP..." << std::endl;
                autoStepActive = false;
                if (sendEmergencyStop(arduino)) {
                    if (!arduino.waitForAckAndDone(500, 500)) {
                        std::cerr << "Warning: No ACK/DONE for E-STOP" << std::endl;
                    }
                } else {
                    std::cerr << "Failed to send E-STOP command" << std::endl;
                }
                std::cout << "Emergency-stop active! Reset Arduino to resume." << std::endl;
                eStopTriggered = true;
                running = false;
            } else if (c == 'v' || c == 'V') {
                // Solve transform
                if (camPoints.size() < 6) {
                    std::cerr << "\nNeed at least 6 points to solve (have "
                              << camPoints.size() << ")" << std::endl;
                } else {
                        if (solveRigidTransform(camPoints, robotPoints, solvedR, solvedT, solvedRms, solvedMax)) {
                            solved = true;
                            std::cout << "\nValidated/Solved transform: RMS=" << solvedRms
                                      << " m, max=" << solvedMax << " m" << std::endl;
                    } else {
                        std::cerr << "\nSolve failed." << std::endl;
                    }
                }
            } else if (c == 'c' || c == 'C') {
                if (!clickState.hasClick) {
                    std::cerr << "\nNo click selected. Click on the pin first." << std::endl;
                    continue;
                }
                if (!hasFrame) {
                    std::cerr << "\nNo camera frame available." << std::endl;
                    continue;
                }
                int srcU = static_cast<int>(std::lround(clickState.lastPx.x * clickState.scaleX));
                int srcV = static_cast<int>(std::lround(clickState.lastPx.y * clickState.scaleY));
                srcU = std::clamp(srcU, 0, lastFrame.depth16.cols - 1);
                srcV = std::clamp(srcV, 0, lastFrame.depth16.rows - 1);
                Eigen::Vector3f camPt;
                if (!pixelToCameraPoint(lastDepthIntr, lastFrame.depth16,
                                        lastFrame.depthScale,
                                        srcU, srcV,
                                        camPt)) {
                    std::cerr << "\nInvalid depth at click. Re-click the pin." << std::endl;
                    continue;
                }

                // Read current angles from Arduino (single snapshot)
                JointAnglesDeg joints;
                int gripper = cfg.gripper.open;
                bool gotAngles = readEepromAngles(arduino, joints, gripper);
                if (!gotAngles) {
                    std::cerr << "\nFailed to read current angles (CMD_GET_ANGLES)." << std::endl;
                    continue;
                }

                Eigen::Vector3f robotPt = forwardKinematics(cfg, joints);

                camPoints.push_back(camPt);
                robotPoints.push_back(robotPt);
                solved = false; // invalidate previous solve

                std::cout << "\nCaptured #" << camPoints.size() << "\n"
                          << "  P_cam:   [" << camPt.x() << ", " << camPt.y() << ", " << camPt.z() << "]\n"
                          << "  P_robot: [" << robotPt.x() << ", " << robotPt.y() << ", " << robotPt.z() << "]" << std::endl;
            } else if (c == 'u' || c == 'U') {
                if (!camPoints.empty()) {
                    camPoints.pop_back();
                    robotPoints.pop_back();
                    solved = false;
                    std::cout << "\nUndo: now " << camPoints.size() << " points." << std::endl;
                }
            } else if (c == 'r' || c == 'R') {
                camPoints.clear();
                robotPoints.clear();
                solved = false;
                std::cout << "\nReset: cleared all points." << std::endl;
            } else if (c == 'w' || c == 'W') {
                if (!solved) {
                    std::cerr << "\nSolve first (press 's') before writing." << std::endl;
                    continue;
                }
                std::filesystem::path outPath = root / "config" / "calibration.yaml";
                if (!writeCalibrationYaml(outPath, solvedR, solvedT, solvedRms, camPoints.size())) {
                    std::cerr << "\nFailed to write calibration to " << outPath << std::endl;
                } else {
                    std::cout << "\nWrote calibration to " << outPath << std::endl;
                }
            } else if (c == 'q' || c == 'Q') {
                running = false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    disableRawMode();

    if (!eStopTriggered) {
        std::cout << "\nReturning to Ready Position..." << std::endl;
        sendMoveAndWait(arduino, cfg, readyJoints, cfg.gripper.open, "ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (haveEeprom) {
            std::cout << "Returning to EEPROM angles..." << std::endl;
            sendMoveAndWait(arduino, cfg, eepromJoints, eepromGripper, "eeprom_restore");
        }
    }

    cam.stop();
    cv::destroyAllWindows();

    std::cout << "\nExiting calibration tool." << std::endl;
    return 0;
}
