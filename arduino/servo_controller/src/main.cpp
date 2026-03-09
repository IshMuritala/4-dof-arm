#include <Arduino.h>
#include  <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdint.h>
#include <SCServo.h>
#include "protocol.h"
#include "servo_limits.h"
#include <EEPROM.h>

//#include <map>



  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



enum ParseState {
    WAITING_FOR_START,
    READING_COMMAND,
    READING_LENGTH,
    READING_DATA,
    READING_CHECKSUM,
    READING_END
};

ParseState parseState = WAITING_FOR_START;
uint8_t rxBuffer[MAX_MESSAGE_LENGTH];
uint8_t bufferIndex = 0;
uint8_t currentCommand = 0;
uint8_t dataLength = 0;
uint8_t receivedChecksum = 0;


SMS_STS sms_sts;

// Base, Shoulder, Elbow, Wrist (start at 90°)
// Gets overwritten by EEPROM on boot, Only used if EEPROM is empty (first boot ever)
int currentServoAngles[4] = {90, 90, 90, 90};
int currentGripperPos = GRIPPER_HOME;
volatile bool eStopActive = false;

#define EEPROM_SERVO_ADDR 0


void sendResponse(uint8_t responseType, const uint8_t* data, uint8_t dataLen) {
    uint8_t buffer[MAX_MESSAGE_LENGTH];
    int index = 0;

    buffer[index++] = START_BYTE;
    buffer[index++] = responseType;
    buffer[index++] = dataLen;

    if (dataLen > 0 && data != nullptr) {
        memcpy(&buffer[index], data, dataLen);
        index += dataLen;
    }

    // Simple checksum (XOR)
    uint8_t checksum = 0;
    for (int i = 1; i < index; i++) {
        checksum ^= buffer[i];
    }
    buffer[index++] = checksum;

    buffer[index++] = END_BYTE;

    Serial.write(buffer, index);
}

// Checksum/length validation
uint8_t computeChecksum(uint8_t cmd, uint8_t len, const uint8_t* data) {
    uint8_t checksum = 0;
    checksum ^= cmd;
    checksum ^= len;
    for (uint8_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

bool isKnownCommand(uint8_t cmd) {
    switch (cmd) {
    case CMD_PING:
    case CMD_MOVE_JOINTS:
    case CMD_HOME:
    case CMD_SHUTDOWN:
    case CMD_EMERGENCY_STOP:
    case CMD_ADJUST_SERVO:
    case CMD_SET_SERVO_ABS:
    case CMD_GET_ANGLES:
        return true;
    default:
        return false;
    }
}

bool validateLength(uint8_t cmd, uint8_t len) {
    switch (cmd) {
    case CMD_PING:
    case CMD_HOME:
    case CMD_SHUTDOWN:
    case CMD_EMERGENCY_STOP:
        return len == 0;
    case CMD_MOVE_JOINTS:
        return len == sizeof(MoveJointsData);
    case CMD_ADJUST_SERVO:
        return len == sizeof(AdjustServoData);
    case CMD_SET_SERVO_ABS:
        return len == sizeof(SetServoAbsData);
    case CMD_GET_ANGLES:
        return len == 0;
    default:
        return false;
    }
}

void sendError(uint8_t errorCode) {
    ErrorData err;
    err.error_code = errorCode;
    sendResponse(RESP_ERROR, (uint8_t*)&err, sizeof(err));
}

bool pollEmergencyStop() {
    static uint8_t state = 0;
    while (Serial.available()) {
        uint8_t b = Serial.read();
        switch (state) {
            case 0: state = (b == START_BYTE) ? 1 : 0; break;
            case 1: state = (b == CMD_EMERGENCY_STOP) ? 2 : 0; break;
            case 2: state = (b == 0) ? 3 : 0; break;
            case 3: state = (b == CMD_EMERGENCY_STOP) ? 4 : 0; break;
            case 4:
                state = 0;
                if (b == END_BYTE) {
                    eStopActive = true;
                    // ACK/DONE immediately so Mac gets confirmation even mid-motion.
                    sendResponse(RESP_ACK, nullptr, 0);
                    sendResponse(RESP_DONE, nullptr, 0);
                    return true;
                }
                break;
            default: state = 0; break;
        }
    }
    return eStopActive;
}

bool isAllowedDuringEStop(uint8_t cmd) {
    return (cmd == CMD_PING) || (cmd == CMD_GET_ANGLES) || (cmd == CMD_EMERGENCY_STOP);
}










bool waitForGripperPosition(int targetPos, int tolerance, int timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
        if (pollEmergencyStop()) {
            return false;
        }
        int pos = sms_sts.ReadPos(GRIPPER_SERVO_ID);
        if (pos >= 0 && abs(pos - targetPos) <= tolerance) {
            currentGripperPos = pos;
            return true;
        }
        delay(30);
    }
    return false;
}

bool homeGripper() {
    int safePos = clampGripperPosition(GRIPPER_HOME);
    sms_sts.WritePosEx(GRIPPER_SERVO_ID, safePos, GRIPPER_SPEED_NORMAL, GRIPPER_ACCEL_NORMAL);
    return waitForGripperPosition(safePos, GRIPPER_HOME_TOL, GRIPPER_MOVE_TIMEOUT_MS);
}






// Convert angle to PCA9685 pulse width
void setServoAngle(uint8_t channel, int angle) {

    int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);


    pwm.setPWM(channel, 0, pulse);
}


uint8_t pwmChannelForServo(uint8_t servoIndex) {
    return SERVO_TO_PWM_CHANNEL[servoIndex];
}

// S-Curve movement (single joint)
void setServoAngleSmooth(uint8_t servoIndex, int targetAngle) {


    int currentAngle = currentServoAngles[servoIndex];


    int distance = targetAngle - currentAngle;


    if (distance == 0) {
        return;
    }

    int steps = abs(distance) * 3;
    int delayTime = 10;

    uint8_t pwmChannel = pwmChannelForServo(servoIndex);
    bool interrupted = false;
    int lastAngle = currentAngle;


    for (int i = 1; i <= steps; i++) {

        int newAngle = currentAngle + (distance * i / steps);


        setServoAngle(pwmChannel, newAngle);
        lastAngle = newAngle;
        currentServoAngles[servoIndex] = newAngle;

        if (pollEmergencyStop()) {
            interrupted = true;
            break;
        }


        delay(delayTime);
    }


    if (!interrupted) {
        currentServoAngles[servoIndex] = targetAngle;
    } else {
        currentServoAngles[servoIndex] = lastAngle;
    }


}

// update all joints each step so they move together
bool moveJointsSmooth(int targetBase, int targetShoulder, int targetElbow, int targetWrist) {
    const int start[4] = {
        currentServoAngles[SERVO_BASE],
        currentServoAngles[SERVO_SHOULDER],
        currentServoAngles[SERVO_ELBOW],
        currentServoAngles[SERVO_WRIST]
    };
    const int target[4] = {targetBase, targetShoulder, targetElbow, targetWrist};
    int dist[4] = {
        target[0] - start[0],
        target[1] - start[1],
        target[2] - start[2],
        target[3] - start[3]
    };

    int maxDist = 0;
    for (int i = 0; i < 4; ++i) {
        int d = abs(dist[i]);
        if (d > maxDist) maxDist = d;
    }
    if (maxDist == 0) {
        return true;
    }

    int steps = maxDist * 3;
    int delayTime = 10;
    for (int i = 1; i <= steps; i++) {
        for (int j = 0; j < 4; ++j) {
            int newAngle = start[j] + (dist[j] * i / steps);
            setServoAngle(pwmChannelForServo(j), newAngle);
            currentServoAngles[j] = newAngle;
        }

        if (pollEmergencyStop()) {
            return false;
        }

        delay(delayTime);
    }

    currentServoAngles[SERVO_BASE] = targetBase;
    currentServoAngles[SERVO_SHOULDER] = targetShoulder;
    currentServoAngles[SERVO_ELBOW] = targetElbow;
    currentServoAngles[SERVO_WRIST] = targetWrist;
    return true;
}

void savePositionsToEEPROM() {

    for (int i = 0; i < 4; i++) {
        int address = EEPROM_SERVO_ADDR + (i * 2);


        EEPROM.write(address, highByte(currentServoAngles[i]));
        EEPROM.write(address + 1, lowByte(currentServoAngles[i]));
    }

    int gripperAddr = EEPROM_SERVO_ADDR + (SERVO_COUNT * 2);
    EEPROM.write(gripperAddr, highByte(currentGripperPos));
    EEPROM.write(gripperAddr + 1, lowByte(currentGripperPos));
}

void loadPositionsFromEEPROM() {

    for (int i = 0; i < 4; i++) {
        int address = EEPROM_SERVO_ADDR + (i * 2);


        byte highB = EEPROM.read(address);
        byte lowB = EEPROM.read(address + 1);
        currentServoAngles[i] = word(highB, lowB);
        currentServoAngles[i] = clampPWMAngle(i, currentServoAngles[i]);
    }

    int gripperAddr = EEPROM_SERVO_ADDR + (SERVO_COUNT * 2);
    byte highG = EEPROM.read(gripperAddr);
    byte lowG = EEPROM.read(gripperAddr + 1);
    currentGripperPos = word(highG, lowG);
    currentGripperPos = clampGripperPosition(currentGripperPos);
}



// ACK = command validated/accepted; DONE = motion loop complete (gripper uses polling)
void executeCommand(uint8_t cmd, uint8_t* data, uint8_t len) {
    if (eStopActive && !isAllowedDuringEStop(cmd)) {
        sendError(ERR_ESTOP);
        return;
    }
    switch (cmd) {
    case CMD_PING:
        //Serial.println("DEBUG: Received PING");  // Commented out - interferes with protocol
        sendResponse(RESP_ACK, nullptr, 0);  // Send ACK back
        break;

    case CMD_MOVE_JOINTS:
        {
            //Serial.println("DEBUG: Received MOVE_JOINTS");  // Commented out - interferes with protocol


            MoveJointsData* moveData = (MoveJointsData*)data;


            int safeBase = clampPWMAngle(SERVO_BASE, moveData->base);
            int safeShoulder = clampPWMAngle(SERVO_SHOULDER, moveData->shoulder);
            int safeElbow = clampPWMAngle(SERVO_ELBOW, moveData->elbow);
            int safeWrist = clampPWMAngle(SERVO_WRIST, moveData->wrist);
            int safeGripper = clampGripperPosition(moveData->gripper);


            sendResponse(RESP_ACK, nullptr, 0);

            // Move PWM servos together (base, shoulder, elbow, wrist)
            if (!moveJointsSmooth(safeBase, safeShoulder, safeElbow, safeWrist)) {
                sendError(ERR_ESTOP);
                return;
            }

            // Move gripper servo (uses different protocol)
            sms_sts.WritePosEx(GRIPPER_SERVO_ID, safeGripper, GRIPPER_SPEED_NORMAL, GRIPPER_ACCEL_NORMAL);
            if (!waitForGripperPosition(safeGripper, GRIPPER_POS_TOL, GRIPPER_MOVE_TIMEOUT_MS)) {
                sendError(eStopActive ? ERR_ESTOP : ERR_TIMEOUT);
                return;
            }
            currentGripperPos = safeGripper;


            sendResponse(RESP_DONE, nullptr, 0);

            break;
        }

    case CMD_ADJUST_SERVO:
        {

            AdjustServoData* adjustData = (AdjustServoData*)data;
            if (adjustData->channel > SERVO_GRIPPER) {
                sendError(ERR_INVALID_CHANNEL);
                break;
            }

            if (adjustData->channel == SERVO_GRIPPER) {
                int newPos = currentGripperPos + adjustData->increment;
                int safePos = clampGripperPosition(newPos);

                sendResponse(RESP_ACK, nullptr, 0);
                sms_sts.WritePosEx(GRIPPER_SERVO_ID, safePos, GRIPPER_SPEED_NORMAL, GRIPPER_ACCEL_NORMAL);
                if (!waitForGripperPosition(safePos, GRIPPER_POS_TOL, GRIPPER_MOVE_TIMEOUT_MS)) {
                    sendError(eStopActive ? ERR_ESTOP : ERR_TIMEOUT);
                    return;
                }
                currentGripperPos = safePos;
                sendResponse(RESP_DONE, nullptr, 0);
                break;
            }


            int currentAngle = currentServoAngles[adjustData->channel];


            int newAngle = currentAngle + adjustData->increment;


            int safeAngle = clampPWMAngle(adjustData->channel, newAngle);


            sendResponse(RESP_ACK, nullptr, 0);


            setServoAngleSmooth(adjustData->channel, safeAngle);

            if (eStopActive) {
                sendError(ERR_ESTOP);
                return;
            }


            sendResponse(RESP_DONE, nullptr, 0);
            break;
        }

    case CMD_SET_SERVO_ABS:
        {
            SetServoAbsData* setData = (SetServoAbsData*)data;

            if (setData->channel > SERVO_GRIPPER) {
                sendError(ERR_INVALID_CHANNEL);
                break;
            }

            if (setData->channel == SERVO_GRIPPER) {
                int safePos = clampGripperPosition(setData->angle);

                sendResponse(RESP_ACK, nullptr, 0);
                sms_sts.WritePosEx(GRIPPER_SERVO_ID, safePos, GRIPPER_SPEED_NORMAL, GRIPPER_ACCEL_NORMAL);
                if (!waitForGripperPosition(safePos, GRIPPER_POS_TOL, GRIPPER_MOVE_TIMEOUT_MS)) {
                    sendError(eStopActive ? ERR_ESTOP : ERR_TIMEOUT);
                    return;
                }
                currentGripperPos = safePos;
                sendResponse(RESP_DONE, nullptr, 0);
                break;
            }

            int safeAngle = clampPWMAngle(setData->channel, setData->angle);


            sendResponse(RESP_ACK, nullptr, 0);

            setServoAngleSmooth(setData->channel, safeAngle);

            if (eStopActive) {
                sendError(ERR_ESTOP);
                return;
            }


            sendResponse(RESP_DONE, nullptr, 0);
            break;
        }


    case CMD_GET_ANGLES:
        {
            GetAnglesData angles;
            angles.base = currentServoAngles[SERVO_BASE];
            angles.shoulder = currentServoAngles[SERVO_SHOULDER];
            angles.elbow = currentServoAngles[SERVO_ELBOW];
            angles.wrist = currentServoAngles[SERVO_WRIST];

            angles.gripper = currentGripperPos;

            sendResponse(RESP_ACK, (uint8_t*)&angles, sizeof(angles));
            break;
        }


    case CMD_HOME:

        //Serial.println("DEBUG: Received HOME");  // Commented out - interferes with protocol


        sendResponse(RESP_ACK, nullptr, 0);


        if (!moveJointsSmooth(BASE_HOME, SHOULDER_HOME, ELBOW_HOME, WRIST_HOME)) {
            sendError(ERR_ESTOP);
            return;
        }


        if (!homeGripper()) {
            sendError(eStopActive ? ERR_ESTOP : ERR_TIMEOUT);
            return;
        }
        currentGripperPos = GRIPPER_HOME;


        sendResponse(RESP_DONE, nullptr, 0);
        break;


    case CMD_EMERGENCY_STOP:
        //Serial.println("DEBUG: Received E-STOP");  // Commented out - interferes with protocol
        eStopActive = true;
        sendResponse(RESP_ACK, nullptr, 0);
        sendResponse(RESP_DONE, nullptr, 0);
        break;

    case CMD_SHUTDOWN:

        sendResponse(RESP_ACK, nullptr, 0);

        if (!homeGripper()) {
            sendError(eStopActive ? ERR_ESTOP : ERR_TIMEOUT);
            return;
        }
        currentGripperPos = GRIPPER_HOME;


        savePositionsToEEPROM();

        sendResponse(RESP_DONE, nullptr, 0);

        //Serial.println("DEBUG: Received SHUTDOWN");  // Commented out - interferes with protocol
        break;

    default:
        // Unknown command - send error
        //Serial.println("DEBUG: Unknown command");  // Commented out - interferes with protocol
        break;
    }
}



void setup() {

    Serial.begin(SERIAL_BAUD_RATE);

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);

    // Initialize gripper servo
    Serial1.begin(1000000);
    sms_sts.pSerial = &Serial1;

    // Load saved positions from EEPROM
    loadPositionsFromEEPROM();

    // Immediately set servos to loaded positions to avoid the startup "snap"
    setServoAngle(pwmChannelForServo(SERVO_BASE), currentServoAngles[0]);
    setServoAngle(pwmChannelForServo(SERVO_SHOULDER), currentServoAngles[1]);
    setServoAngle(pwmChannelForServo(SERVO_ELBOW), currentServoAngles[2]);
    setServoAngle(pwmChannelForServo(SERVO_WRIST), currentServoAngles[3]);

    // delay to let PWM settle and the serial bus come up before gripper read
    delay(500);

    int gripperPos = sms_sts.ReadPos(GRIPPER_SERVO_ID);
    if (verifyGripperHome(gripperPos)) {
        currentGripperPos = gripperPos;
    } else if (homeGripper()) {
        currentGripperPos = GRIPPER_HOME;
    } else {
        currentGripperPos = clampGripperPosition(gripperPos);
    }


    Serial.println("Arduino ready for commands");


    // TEMPORARY TEST: Auto-home on startup for testing
    //Serial.println("Auto-homing servos...");
    //setServoAngleSmooth(SERVO_BASE, BASE_HOME);
    //setServoAngleSmooth(SERVO_SHOULDER, SHOULDER_HOME);
    //setServoAngleSmooth(SERVO_ELBOW, ELBOW_HOME);
    //setServoAngleSmooth(SERVO_WRIST, WRIST_HOME);
    //delay(2000);
    //Serial.println("Homing complete!");
}


void loop() {

    if (Serial.available()) {
        uint8_t incomingByte = Serial.read();

        switch (parseState)
        {
            case WAITING_FOR_START:
                if (incomingByte == START_BYTE)
                {
                    parseState = READING_COMMAND;
                }
            break;

            case READING_COMMAND:
            currentCommand = incomingByte;
            parseState = READING_LENGTH;
            break;

        case READING_LENGTH:
            dataLength = incomingByte;
            bufferIndex = 0;

            if (!isKnownCommand(currentCommand)) {
                sendError(ERR_UNKNOWN_COMMAND);
                parseState = WAITING_FOR_START;
                break;
            }

            if (dataLength > MAX_MESSAGE_LENGTH) {
                sendError(ERR_INVALID_LENGTH);
                parseState = WAITING_FOR_START;
                break;
            }

            if (!validateLength(currentCommand, dataLength)) {
                sendError(ERR_INVALID_LENGTH);
                parseState = WAITING_FOR_START;
                break;
            }

            if (dataLength == 0) {
                parseState = READING_CHECKSUM;
            } else {
                parseState = READING_DATA;
            }
            break;

        case READING_DATA:
            if (bufferIndex >= MAX_MESSAGE_LENGTH) {
                sendError(ERR_INVALID_LENGTH);
                parseState = WAITING_FOR_START;
                break;
            }
            rxBuffer[bufferIndex++] = incomingByte;
            if (bufferIndex >= dataLength) {
                parseState = READING_CHECKSUM;
            }
            break;

            case READING_CHECKSUM:
            receivedChecksum = incomingByte;
            parseState = READING_END;
            break;

        case READING_END:
            if (incomingByte == END_BYTE)
            {
                uint8_t expected = computeChecksum(currentCommand, dataLength, rxBuffer);
                if (receivedChecksum != expected) {
                    sendError(ERR_INVALID_CHECKSUM);
                } else {
                    executeCommand(currentCommand, rxBuffer, dataLength);
                }
            }

            parseState = WAITING_FOR_START;
            break;


        }
    }
}



