
#ifndef SERVO_LIMITS_H
#define SERVO_LIMITS_H

// PWM SERVOS ( via PCA9685)
// Pulse width limits
#define SERVOMIN  102   // pulse = 0 degrees
#define SERVOMAX  512   // pulse = 180 degrees
#define SERVO_FREQ 50   // 50Hz for analog servos

// Servo channel assignments on PCA9685
#define BASE_CHANNEL     0
#define SHOULDER_CHANNEL 2
#define ELBOW_CHANNEL    4
#define WRIST_CHANNEL    6

// Logical servo indices (for protocol + tracking array)
#define SERVO_BASE     0
#define SERVO_SHOULDER 1
#define SERVO_ELBOW    2
#define SERVO_WRIST    3
#define SERVO_COUNT    4   // PWM joints only
#define SERVO_GRIPPER  4   // protocol index for gripper

// Map logical servo index -> PCA9685 channel
static const uint8_t SERVO_TO_PWM_CHANNEL[SERVO_COUNT] = {
    BASE_CHANNEL, SHOULDER_CHANNEL, ELBOW_CHANNEL, WRIST_CHANNEL
};

// Home positions
 #define BASE_HOME        90  // Center/forward pos
 #define SHOULDER_HOME    90  //
 #define ELBOW_HOME       90  //
 #define WRIST_HOME       90  //

// Per-servo safe ranges
#define BASE_MIN     0
#define BASE_MAX     180

#define SHOULDER_MIN 0
#define SHOULDER_MAX 180

#define ELBOW_MIN    0
#define ELBOW_MAX    150

#define WRIST_MIN    48
#define WRIST_MAX    115

/*
^   NOTE: Remember to update servoMin and servoMax in manual_control.cpp whenever
    ANY Per-servo safe ranges are changed
    THEY HAVE TO MATCH!

---
const int servoMin[SERVO_COUNT] = {0, 0, 0, 0, 1};
const int servoMax[SERVO_COUNT] = {180, 180, 180, 180, 3700};
---
v: Same idea for the gripper servo too if ever changing that.

NOTE: Also make the same changes in calibration.cpp
NOTE: in changning gipper home position, remember to also change 'GRIPPER_HOME=' in both files
*/


// GRIPPER SERVO (STS3215 - via Serial Bus Board)
#define GRIPPER_SERVO_ID    1

 // operating range
 #define GRIPPER_OPEN        1000   // Fully open position // OLD: 1
 #define GRIPPER_CLOSED      4000  // Fully closed  // OLD: 3700

 // Homing configuration
 #define GRIPPER_HOME        2800  // Safe middle pos // OLD: 2000
 #define GRIPPER_HOME_TOL    50   // +/- 50 positions acceptable on startup --

 // Movement parameters (speed, acceleration)
 #define GRIPPER_SPEED_NORMAL  1000  // speed (0-4000, lower=faster)
 #define GRIPPER_ACCEL_NORMAL  50    // acceleration (0-254, lower=faster)

// Gripper polling
#define GRIPPER_POS_TOL          50 //  --
#define GRIPPER_MOVE_TIMEOUT_MS  8000




inline int clampPWMAngle(int servoIndex, int angle)
{
    int minA = 0;
    int maxA = 180;

    switch (servoIndex) {
        case SERVO_BASE:     minA = BASE_MIN;     maxA = BASE_MAX;     break;
        case SERVO_SHOULDER: minA = SHOULDER_MIN; maxA = SHOULDER_MAX; break;
        case SERVO_ELBOW:    minA = ELBOW_MIN;    maxA = ELBOW_MAX;    break;
        case SERVO_WRIST:    minA = WRIST_MIN;    maxA = WRIST_MAX;    break;
        default: break;
    }

    return constrain(angle, minA, maxA);
}


inline int clampGripperPosition(int pos)
{
    return constrain(pos, GRIPPER_OPEN, GRIPPER_CLOSED);
}



inline bool verifyGripperHome(int currentPosition) {

    return (currentPosition >= GRIPPER_HOME - GRIPPER_HOME_TOL) &&
           (currentPosition <= GRIPPER_HOME + GRIPPER_HOME_TOL);
}


#endif // SERVO_LIMITS_H
