#include "ParallaxDriver.h"

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

namespace PPCCommand {
    enum command_t {
        CMD_QPOS = 0, // query position
        CMD_QSPD, // query speed (avg)
        CMD_CHFA, // check for arrival
        CMD_TRVL, // travel to position
        CMD_CLRP, // clear position
        CMD_SREV, // set orientation as reversed
        CMD_STXD, // set transmission delay
        CMD_SMAX, // set maximum speed
        CMD_SSRR // set speed ramp rate
    };

    static const char *names[] {
        "QPOS",
        "QSPD",
        "CHFA",
        "TRVL",
        "CLRP",
        "SREV",
        "STXD",
        "SMAX",
        "SSRR"
    };

    static const uint8_t cmd[] = {0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48};
    static const uint8_t len_send[] = {0, 0, 0, 1, 2, 0, 0, 1, 2, 1};
    static const uint8_t len_recv[] = {2, 2, 2, 1, 0, 0, 0, 0, 0, 0};
};

ParallaxPositionController::ParallaxPositionController(serial::Serial * ser, uint8_t id) {
    if (ser == NULL) {
        ROS_ERROR("Serial port is NULL");
    }
    m_serial = ser;
    m_id = id;
}

ParallaxPositionController::~ParallaxPositionController() {
}

void ParallaxPositionController::open() {
    // configure serial for 19200 8-N-1
    m_serial->setBaudrate(BAUDRATE);
    m_serial->setStopbits(serial::stopbits_one);
    m_serial->setParity(serial::parity_none);
    m_serial->setBytesize(serial::eightbits);
    m_serial->setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    m_serial->setTimeout(timeout);
    ROS_INFO("Serial port %s configured", this->getPortName().c_str());

    if (!m_serial->isOpen()) {
        m_serial->open();
        ROS_INFO("Serial port %s opened", this->getPortName().c_str());
    } else {
        ROS_INFO("Serial port %s already opened", this->getPortName().c_str());
    }
}

void ParallaxPositionController::close() {
    if (m_serial->isOpen()) {
        m_serial->close();
        ROS_INFO("Serial port %s closed", this->getPortName().c_str());
    } else {
        ROS_INFO("Serial port %s already closed", this->getPortName().c_str());
    }
}

std::string ParallaxPositionController::getPortName() {
    return m_serial->getPort();
}

serial::Serial * ParallaxPositionController::getSerialPort() {
    return m_serial;
}

void ParallaxPositionController::reset() {

}

void ParallaxPositionController::initialize() {
    ROS_INFO("Initializing Parallax Position Controller with id 0x%x", m_id);
}

char* ParallaxPositionController::byteArrayToString(uint8_t * buf, uint8_t buflen) {
    char * str = reinterpret_cast<char*>(malloc(buflen * 5 + 3));
    memset(str, 0, buflen * 5 + 1);

    char * ptr = str + 1;
    for (uint8_t i = 0; i < buflen; i++) {
        ptr += sprintf(ptr, "0x%02x ", buf[i]);
    }
    str[0] = '[';
    str[buflen * 5 + 1] = ']';
    str[buflen * 5 + 2] = '\0';
    return str;
}

size_t ParallaxPositionController::transaction(uint8_t id, uint8_t cmd, uint8_t * sendbuf, 
        uint8_t * readbuf) {
    char * datastr = byteArrayToString(sendbuf, PPCCommand::len_send[cmd]);

    ROS_DEBUG("Starting transaction to id 0x%x, with command [%d: %s] and data %s", 
            id, cmd, PPCCommand::names[cmd], datastr);
    free(datastr);

    // create send buffer
    uint8_t * buf = reinterpret_cast<uint8_t*>(malloc(PPCCommand::len_send[cmd] + 1));
    buf[0] = id | PPCCommand::cmd[cmd];

    if (PPCCommand::len_send[cmd] > 0 && sendbuf != NULL) {
        memcpy(buf + 1, sendbuf, PPCCommand::len_send[cmd]);
    }

    // send the command
    size_t bytes_sent = m_serial->write(buf, PPCCommand::len_send[cmd] + 1);
    if (bytes_sent != PPCCommand::len_send[cmd] + 1) {
        ROS_WARN("Not all bytes successfully sent");
    }

    // wait for a response
    size_t bytes_recv = 0;
    if (PPCCommand::len_recv[cmd] > 0 && readbuf != NULL) {
        bytes_recv = m_serial->read(readbuf, PPCCommand::len_recv[cmd]);
        datastr = byteArrayToString(readbuf, bytes_recv);
        ROS_DEBUG("Received %d/%d byte%s: %s", (int) bytes_recv, PPCCommand::len_recv[cmd],
                (bytes_recv == 1) ? "s" : "", datastr);
        free(datastr);
    }

    if (bytes_recv != PPCCommand::len_recv[cmd]) {
        ROS_WARN("Not all bytes successfully read");
    }

    ROS_DEBUG("Transaction to id 0x%x with command [%d: %s] completed", 
            id, cmd, PPCCommand::names[cmd]);

    return bytes_recv;
}

int16_t ParallaxPositionController::construct_int16(uint8_t * buf) {
    // assume high byte first
    return (buf[0] << 8u) | buf[1];
}
