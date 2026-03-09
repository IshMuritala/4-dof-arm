
#ifndef INC_4DOF_ARM_CAMERA_CONFIG_H
#define INC_4DOF_ARM_CAMERA_CONFIG_H

// Depth preview / filtering range (millimeters)

// Absolute Max seems to be ~10,000 | ~200 is recommended for absolute Min based on docs. ~150 is Absolut min.
// Can go lower than ~150 BUT closer pixels usually come back as 0
constexpr int DEPTH_MIN_MM = 50; // old value: 120 //  50
constexpr int DEPTH_MAX_MM = 650; // old value: 600 // 800


// NOTE: Issues with all preprocessing, disabled for now.
// Workspace crop (meters) in camera frame: X left/right, Y up/down, Z forward
constexpr bool ENABLE_WORKSPACE_CROP = false;
constexpr float WORKSPACE_MIN_X_M = -3.60f;
constexpr float WORKSPACE_MAX_X_M = 3.60f;
constexpr float WORKSPACE_MIN_Y_M = -2.60f;
constexpr float WORKSPACE_MAX_Y_M = 0.60f;
constexpr float WORKSPACE_MIN_Z_M = DEPTH_MIN_MM / 1000.0f; // Don't edit this Z directly | Change DEPTH Min/Max
constexpr float WORKSPACE_MAX_Z_M = DEPTH_MAX_MM / 1000.0f; // Don't edit this Z directly | Change DEPTH Min/Max

// Voxel downsample size (meters). Smaller = more detail, larger = faster
constexpr bool ENABLE_VOXEL_DOWNSAMPLE = false;
constexpr float VOXEL_LEAF_SIZE_M = 0.002f; // 0.003f

// Statistical outlier removal
// OUTLIER_MEAN_K: Larger = more stable but slower | Smaller = faster but noisier. (min: ~10, max: ~ 50 - 100)
// OUTLIER_STDDEV_MUL: Smaller value = more aggressive point removal | Larger value = more lenient (min: 0, max: ~3+)
constexpr bool ENABLE_OUTLIER_REMOVAL = false;
constexpr int OUTLIER_MEAN_K = 24;
constexpr float OUTLIER_STDDEV_MUL = 1.0f;

#endif //INC_4DOF_ARM_CAMERA_CONFIG_H
