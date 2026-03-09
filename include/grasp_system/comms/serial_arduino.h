#ifndef INC_4DOF_ARM_SERIAL_ARDUINO_H
#define INC_4DOF_ARM_SERIAL_ARDUINO_H
#include <stdint.h>
#include <string>
#include "protocol/protocol.h"

class SerialArduino
{
public:
    SerialArduino();
    ~SerialArduino();

    // Connection management
    bool connect(const std::string& port, int baudRate = SERIAL_BAUD_RATE);
    bool connectAuto(int baudRate = SERIAL_BAUD_RATE);
    void disconnect();
    bool isConnected() const;
    void flushInput();

    // Command sending
    bool sendPing();
    bool sendMoveJoints(uint16_t base, uint16_t shoulder, uint16_t elbow, uint16_t wrist, uint16_t gripper);
    bool sendHome();
    bool sendEmergencyStop();
    bool sendShutdown();
    bool sendSetServoAbs(uint8_t channel, uint16_t angle);
    bool sendGetAngles();


    bool sendCommand(uint8_t cmd, const uint8_t* data, uint8_t dataLen);
    bool waitForResponse(uint8_t& responseType, uint8_t* data, uint8_t& dataLen, int timeoutMs = 1000);

    bool waitForAckAndDone(int ackTimeoutMs = 1000, int doneTimeoutMs = 1000);

    private:
        int fd_; // File descriptor for serial port
        bool connected_;

        uint8_t calculateChecksum(const uint8_t* buffer, int length);

};


#endif //INC_4DOF_ARM_SERIAL_ARDUINO_H
