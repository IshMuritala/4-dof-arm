#ifndef INC_4DOF_ARM_PROTOCOL_H
#define INC_4DOF_ARM_PROTOCOL_H

#include <stdint.h>

// Serial config (shared between Mac and Arduino)
#define SERIAL_BAUD_RATE 9600

// Commands from Mac --> Arduino
// CMD_MOVE_JOINTS: planner/IK path
#define CMD_MOVE_JOINTS 0x01
// CMD_SET_SERVO_ABS: for manual control
#define CMD_SET_SERVO_ABS 0x06

#define CMD_PING 0x00
#define CMD_HOME 0x02
#define CMD_SHUTDOWN 0x03
#define CMD_EMERGENCY_STOP 0x04
#define CMD_ADJUST_SERVO 0x05
#define CMD_GET_ANGLES 0x07 // info


// Responses from Arduino --> Mac
#define RESP_ACK 0x10
#define RESP_DONE 0x11
#define RESP_ERROR 0x12

// Message framing bytes
#define START_BYTE 0xAA
#define END_BYTE 0xFF

// Buffer sizes
#define MAX_MESSAGE_LENGTH 32

// Error codes
#define ERR_UNKNOWN_COMMAND 0x20
#define ERR_INVALID_CHECKSUM 0x21
#define ERR_INVALID_LENGTH 0x22
#define ERR_SERVO_LIMIT 0x23
#define ERR_TIMEOUT 0x24
#define ERR_INVALID_CHANNEL 0x25
#define ERR_ESTOP 0x26


struct ErrorData
{
    uint8_t error_code; // One of the ERR_* codes
};

struct AckData
{
    uint8_t acked_command;
};



struct MoveJointsData {
    uint16_t base;      // 0-180 degrees
    uint16_t shoulder;  // 0-180 degrees
    uint16_t elbow;     // 0-180 degrees
    uint16_t wrist;     // 0-180 degrees
    uint16_t gripper;   // 0-4095 position
};


struct AdjustServoData {
    uint8_t channel;      // 0-3 joints, 4=gripper
    int8_t increment;     // degrees for joints, position units for gripper
};

struct SetServoAbsData {
    uint8_t channel;
    uint16_t angle;    // 0-180 for joints, 0-4095 position for gripper
};


struct GetAnglesData {
    uint16_t base;
    uint16_t shoulder;
    uint16_t elbow;
    uint16_t wrist;
    uint16_t gripper;
};

#endif //INC_4DOF_ARM_PROTOCOL_H
