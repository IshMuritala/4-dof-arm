
#include "grasp_system/comms/serial_arduino.h"
#include "protocol/protocol.h"

#include <fcntl.h>      // For open()
#include <unistd.h>     // For read(), write(), close()
#include <termios.h>    // For serial port configuration
#include <cstring>      // For memset, memcpy
#include <iostream>     // For debug prints

#include <errno.h>

#include <dirent.h>
#include <vector>
#include <algorithm>




namespace {
    bool baudRateToTermios(int baudRate, speed_t& out) {
        switch (baudRate) {
            case 9600: out = B9600; return true;
            case 19200: out = B19200; return true;
            case 38400: out = B38400; return true;
            case 57600: out = B57600; return true;
            case 115200: out = B115200; return true;
        #ifdef B230400
            case 230400: out = B230400; return true;
        #endif
        #ifdef B460800
            case 460800: out = B460800; return true;
        #endif
        #ifdef B921600
            case 921600: out = B921600; return true;
        #endif
        #ifdef B1000000
            case 1000000: out = B1000000; return true;
        #endif
            default:
                return false;
        }
    }

    std::vector<std::string> listSerialPorts() {
        std::vector<std::string> matches;
        DIR* dir = opendir("/dev");
        if (!dir) return matches;

        dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            if (name.rfind("cu.usbmodem", 0) == 0 ||
                name.rfind("cu.usbserial", 0) == 0 ||
                name.rfind("tty.usbmodem", 0) == 0 ||
                name.rfind("tty.usbserial", 0) == 0) {
                matches.push_back("/dev/" + name);
                }
        }
        closedir(dir);

        std::sort(matches.begin(), matches.end());
        return matches;
    }

    std::string pickDefaultPort() {
        auto ports = listSerialPorts();
        for (const auto& p : ports) {
            if (p.rfind("/dev/cu.", 0) == 0) {
                return p; // prefer /dev/cu.* for outbound serial
            }
        }
        if (ports.empty()) return "";
        return ports.front();
    }
} // namespace


SerialArduino::SerialArduino() : fd_(-1), connected_(false)
{

}

SerialArduino::~SerialArduino()
{
    disconnect();
}

bool SerialArduino::isConnected() const
{
    return connected_;
}

void SerialArduino::flushInput()
{
    if (connected_) {
        tcflush(fd_, TCIFLUSH);
    }
}

void SerialArduino::disconnect()
{
    if (connected_) {
        close(fd_);  // Close the file descriptor
        fd_ = -1;
        connected_ = false;
        std::cout << "Disconnected from Arduino" << std::endl;
    }
}

bool SerialArduino::connectAuto(int baudRate)
{
    std::string port = pickDefaultPort();
    if (port.empty()) {
        std::cerr << "No serial devices found in /dev (usbmodem/usbserial)." << std::endl;
        return false;
    }
    return connect(port, baudRate);
}

bool SerialArduino::connect(const std::string& port, int baudRate)
{
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1)
    {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return false;
    }

    // Make file descriptor blocking
    fcntl(fd_, F_SETFL, 0);


    struct termios options;
    tcgetattr(fd_, &options);  // Get current settings


    speed_t speed;
    if (!baudRateToTermios(baudRate, speed)) {
        std::cerr << "Unsupported baud rate " << baudRate
                  << ", defaulting to " << SERIAL_BAUD_RATE << std::endl;
        baudRateToTermios(SERIAL_BAUD_RATE, speed);
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8N1 mode
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Raw mode (no processing)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable flow control
    options.c_cflag |= CREAD | CLOCAL;  // Enable receiver, ignore modem status



    // Apply settings
    tcsetattr(fd_, TCSANOW, &options);

    // Give Arduino time to reset after connection
    sleep(2);

    // Flush any text Arduino printed during boot
    tcflush(fd_, TCIFLUSH);

    connected_ = true;
    std::cout << "Connected to Arduino on " << port << std::endl;

    return true;


}

uint8_t SerialArduino::calculateChecksum(const uint8_t* buffer, int length)
{
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++)
    {
        checksum ^= buffer[i];
    }
    return checksum;

}

bool SerialArduino::sendCommand(uint8_t cmd, const uint8_t* data, uint8_t dataLen)
{
    if (!connected_)
    {
        std::cerr << "Not connected to Arduino" << std::endl;
        return false;
    }


    uint8_t buffer[MAX_MESSAGE_LENGTH];
    int index = 0;

    buffer[index++] = START_BYTE;
    buffer[index++] = cmd;
    buffer[index++] = dataLen;

    // Copy data payload
    if (dataLen > 0 && data != nullptr) {
        memcpy(&buffer[index], data, dataLen);
        index += dataLen;
    }



    uint8_t checksum = calculateChecksum(&buffer[1], index - 1);
    buffer[index++] = checksum;

    buffer[index++] = END_BYTE;


    int bytesWritten = write(fd_, buffer, index);
    std::cout << "DEBUG: Tried to write " << index << " bytes, wrote " << bytesWritten << std::endl;
    if (bytesWritten < 0) {
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    if (bytesWritten != index) {
        std::cerr << "Failed to write complete message" << std::endl;
        return false;
    }


    std::cout << "Sent command 0x" << std::hex << (int)cmd << std::dec << std::endl;
    return true;


}


bool SerialArduino::sendPing()
{

    return sendCommand(CMD_PING, nullptr, 0);
}


bool SerialArduino::sendHome()
{

    return sendCommand(CMD_HOME, nullptr, 0);
}


bool SerialArduino::sendEmergencyStop()
{

    return sendCommand(CMD_EMERGENCY_STOP, nullptr, 0);
}


bool SerialArduino::sendShutdown()
{

    return sendCommand(CMD_SHUTDOWN, nullptr, 0);
}

bool SerialArduino::sendMoveJoints(uint16_t base, uint16_t shoulder, uint16_t elbow, uint16_t wrist, uint16_t gripper)
{

    MoveJointsData data;
    data.base = base;
    data.shoulder = shoulder;
    data.elbow = elbow;
    data.wrist = wrist;
    data.gripper = gripper;


    return sendCommand(CMD_MOVE_JOINTS, (uint8_t*)&data, sizeof(MoveJointsData));
}


bool SerialArduino::sendSetServoAbs(uint8_t channel, uint16_t angle)
{
    SetServoAbsData data;
    data.channel = channel;
    data.angle = angle;
    return sendCommand(CMD_SET_SERVO_ABS, (uint8_t*)&data, sizeof(SetServoAbsData));
}

bool SerialArduino::sendGetAngles()
{
    return sendCommand(CMD_GET_ANGLES, nullptr, 0);
}



bool SerialArduino::waitForResponse(uint8_t& responseType, uint8_t* data, uint8_t& dataLen, int timeoutMs)
{
    if (!connected_) {
        std::cerr << "Not connected to Arduino" << std::endl;
        return false;
    }


    fd_set set;
    struct timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    uint8_t buffer[MAX_MESSAGE_LENGTH];
    int index = 0;
    bool started = false;



    while (index < MAX_MESSAGE_LENGTH) {
        FD_ZERO(&set);
        FD_SET(fd_, &set);

        int rv = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
        if (rv == 0) {
            std::cerr << "Timeout waiting for response" << std::endl;
            return false;
        } else if (rv < 0) {
            std::cerr << "Error reading from serial port" << std::endl;
            return false;
        }

        uint8_t byte;
        int bytesRead = read(fd_, &byte, 1);
        if (bytesRead != 1) continue;

        std::cout << "DEBUG RX: 0x" << std::hex << (int)byte << std::dec << std::endl; // DEBUG: Print every byte received from Arduino

        if (!started) {
            if (byte == START_BYTE) {
                buffer[0] = byte;
                index = 1;
                started = true;
            }
            continue;
        }

        if (byte == START_BYTE) {
            buffer[0] = byte;
            index = 1;
            continue;
        }

        buffer[index++] = byte;


        if (index >= 5 && buffer[index-1] == END_BYTE && buffer[0] == START_BYTE) {

            responseType = buffer[1];
            dataLen = buffer[2];


            if (dataLen > 0 && data != nullptr) {
                memcpy(data, &buffer[3], dataLen);
            }

            std::cout << "Received response 0x" << std::hex << (int)responseType << std::dec << std::endl;
            return true;
        }
    }

    std::cerr << "Buffer overflow while reading response" << std::endl;
    return false;
}

bool SerialArduino::waitForAckAndDone(int ackTimeoutMs, int doneTimeoutMs)
{
    uint8_t responseType = 0;
    uint8_t responseData[MAX_MESSAGE_LENGTH];
    uint8_t responseLen = 0;

    if (!waitForResponse(responseType, responseData, responseLen, ackTimeoutMs)) {
        return false;
    }
    if (responseType != RESP_ACK) {
        if (responseType == RESP_ERROR && responseLen >= 1) {
            std::cerr << "Received ERROR after command: 0x"
                      << std::hex << (int)responseData[0] << std::dec << std::endl;
        }
        std::cerr << "Expected ACK, got 0x" << std::hex << (int)responseType << std::dec << std::endl;
        return false;
    }

    if (!waitForResponse(responseType, responseData, responseLen, doneTimeoutMs)) {
        return false;
    }
    if (responseType == RESP_ERROR && responseLen >= 1) {
        std::cerr << "Received ERROR after ACK: 0x"
                  << std::hex << (int)responseData[0] << std::dec << std::endl;
        return false;
    }
    if (responseType != RESP_DONE) {
        std::cerr << "Expected DONE, got 0x" << std::hex << (int)responseType << std::dec << std::endl;
        return false;
    }

    return true;
}
