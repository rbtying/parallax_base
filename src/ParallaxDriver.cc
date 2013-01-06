#include "ParallaxDriver.h"

#include <string>
#include <iostream>
#include <serial/serial.h>

ParallaxPositionController::ParallaxPositionController(serial::Serial * ser, uint8_t id) {
    if (ser == NULL) {
        std::cerr << "Serial port is NULL" << std::endl;
    }
    m_serial = ser;
    m_id = id;
}

ParallaxPositionController::~ParallaxPositionController() {
}

void ParallaxPositionController::open() {
    if (!m_serial->isOpen()) {
        m_serial->open();
    }

    // configure serial for 19200 8-N-1
    m_serial->setBaudrate(BAUDRATE);
    m_serial->setStopbits(serial::stopbits_one);
    m_serial->setParity(serial::parity_none);
    m_serial->setBytesize(serial::eightbits);
    m_serial->setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    m_serial->setTimeout(timeout);
}

void ParallaxPositionController::close() {
    if (m_serial->isOpen()) {
        m_serial->close();
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

}

size_t ParallaxPositionController::transaction(uint8_t id, uint8_t cmd, uint8_t numToSend,
        uint8_t sendbuf, uint8_t numToRead, uint8_t * readbuf) {
    
}
