cmake_minimum_required(VERSION 4.0)
project(4DOF_ARM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Workaround for Eigen NEON intrinsics errors on macOS CLT clang
add_compile_definitions(EIGEN_DONT_VECTORIZE)


# FIND LIBRARIES:
# Orbbec SDK location
set(OrbbecSDK_DIR "Path to OrbbecSDK lib directory")

# Find required libraries
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED) # (required by VTK)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation visualization)
find_package(Eigen3 REQUIRED)

# INCLUDE DIRECTORIES:
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/shared
        ${OrbbecSDK_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
)

# BUILD EXECUTABLE:
add_executable(4DOF_ARM
        include/grasp_system/core/types.h
        include/grasp_system/core/logger.h
        include/grasp_system/core/state_machine.h
        include/grasp_system/camera/orbbec_camera.h
        include/grasp_system/perception/pointcloud_builder.h
        include/grasp_system/perception/segmentation.h
        include/grasp_system/perception/features_color.h
        include/grasp_system/perception/tracking.h
        include/grasp_system/perception/scene_summary.h
        include/grasp_system/planning/ik.h
        include/grasp_system/planning/primitives.h
        include/grasp_system/planning/safety_checks.h
        include/grasp_system/planning/robot_config.h
        include/grasp_system/planning/fk.h
        include/grasp_system/planning/calibration.h
        include/grasp_system/app/vision_viewer.h
        include/grasp_system/comms/serial_arduino.h
        include/grasp_system/viz/pcl_viewer.h
        include/grasp_system/viz/overlays.h
        src/main.cpp
        src/core/logger.cpp
        src/core/state_machine.cpp
        src/camera/orbbec_camera.cpp
        src/perception/pointcloud_builder.cpp
        src/perception/segmentation.cpp
        src/perception/features_color.cpp
        src/perception/scene_summary.cpp
        src/perception/tracking.cpp
        src/planning/primitives.cpp
        src/planning/ik.cpp
        src/planning/safety_checks.cpp
        src/planning/robot_config.cpp
        src/planning/fk.cpp
        src/planning/calibration.cpp
        src/app/vision_viewer.cpp
        src/comms/serial_arduino.cpp
        src/viz/pcl_viewer.cpp
        src/viz/overlays.cpp
        shared/protocol/protocol.h

)


# LINK LIBRARIES:
target_link_libraries(4DOF_ARM
        ob::OrbbecSDK
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        Eigen3::Eigen
)





# Test program for serial communication
add_executable(test_serial
        src/test_serial.cpp
        src/comms/serial_arduino.cpp
        include/grasp_system/comms/serial_arduino.h
        shared/protocol/protocol.h
)

target_link_libraries(test_serial)

# Manual servo control
add_executable(manual_control
        src/manual_control.cpp
        src/comms/serial_arduino.cpp
        include/grasp_system/comms/serial_arduino.h
        shared/protocol/protocol.h
)

target_link_libraries(manual_control)

# Helper: print Orbbec intrinsics
add_executable(test_orbbec_intrinsics
        src/test_orbbec_intrinsics.cpp
)

target_link_libraries(test_orbbec_intrinsics
        ob::OrbbecSDK
)

# Helper: capture a single color + depth frame to /data
add_executable(test_orbbec_capture
        src/test_orbbec_capture.cpp
        src/camera/orbbec_camera.cpp
        include/grasp_system/camera/orbbec_camera.h
)

target_link_libraries(test_orbbec_capture
        ob::OrbbecSDK
        ${OpenCV_LIBS}
)

# Helper: live PCL viewer for aligned clouds
add_executable(test_pcl_viewer
        src/test_pcl_viewer.cpp
        src/camera/orbbec_camera.cpp
        src/perception/pointcloud_builder.cpp
        src/perception/segmentation.cpp
        src/viz/pcl_viewer.cpp
        include/grasp_system/camera/orbbec_camera.h
        include/grasp_system/perception/pointcloud_builder.h
        include/grasp_system/perception/segmentation.h
        include/grasp_system/viz/pcl_viewer.h
)

target_link_libraries(test_pcl_viewer
        ob::OrbbecSDK
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# Helper: plane segmentation test
add_executable(test_segmentation
        src/test_segmentation.cpp
        src/app/vision_viewer.cpp
        src/camera/orbbec_camera.cpp
        src/perception/pointcloud_builder.cpp
        src/perception/segmentation.cpp
        src/perception/features_color.cpp
        src/perception/scene_summary.cpp
        src/perception/tracking.cpp
        src/planning/calibration.cpp
        src/viz/pcl_viewer.cpp
        include/grasp_system/camera/orbbec_camera.h
        include/grasp_system/perception/pointcloud_builder.h
        include/grasp_system/perception/segmentation.h
        include/grasp_system/perception/features_color.h
        include/grasp_system/perception/scene_summary.h
        include/grasp_system/perception/tracking.h
        include/grasp_system/planning/calibration.h
        include/grasp_system/app/vision_viewer.h
        include/grasp_system/viz/pcl_viewer.h
)

target_link_libraries(test_segmentation
        ob::OrbbecSDK
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# Helper: camera-to-robot calibration
add_executable(calibration
        src/calibration.cpp
        src/camera/orbbec_camera.cpp
        src/planning/robot_config.cpp
        src/planning/fk.cpp
        src/comms/serial_arduino.cpp
        include/grasp_system/camera/orbbec_camera.h
        include/grasp_system/planning/robot_config.h
        include/grasp_system/planning/fk.h
        include/grasp_system/comms/serial_arduino.h
        shared/protocol/protocol.h
)

target_link_libraries(calibration
        ob::OrbbecSDK
        ${OpenCV_LIBS}
        Eigen3::Eigen
)

# Temporary: primitive execution test
add_executable(test_primitives
        src/test_primitives.cpp
        src/planning/primitives.cpp
        src/planning/ik.cpp
        src/planning/safety_checks.cpp
        src/planning/robot_config.cpp
        include/grasp_system/planning/primitives.h
        include/grasp_system/planning/ik.h
        include/grasp_system/planning/safety_checks.h
        include/grasp_system/planning/robot_config.h
        include/grasp_system/planning/fk.h
        src/planning/fk.cpp
        src/comms/serial_arduino.cpp
        include/grasp_system/comms/serial_arduino.h
        shared/protocol/protocol.h
)

target_link_libraries(test_primitives
        ${OpenCV_LIBS}
        Eigen3::Eigen
)
