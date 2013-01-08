#include "VelocityController.h"
#include <ros/ros.h>
#include <cmath>

const double VelocityController::TICKS_PER_ROTATION = 144;
const double VelocityController::PI = 3.1415926536;
const double VelocityController::TICKS_TO_RADIANS = 2.0 * PI / TICKS_PER_ROTATION;
const double VelocityController::RADIANS_TO_TICKS = 1.0 / TICKS_TO_RADIANS;

VelocityController::VelocityController(std::string portname, uint8_t * ids, 
        uint8_t idcount, double rate) : m_control(portname, ids, idcount) {
    m_control.open();
    m_rate = rate;
    m_set_speed = new double[idcount];
    m_desired_speed = new double[idcount];
    m_accel = new double[idcount];
    m_current_speed = new int16_t[idcount];
    m_current_pos = new int16_t[idcount];
    m_num_motors = idcount;

    initialize();
}

VelocityController::~VelocityController() {
    delete[] m_set_speed;
    delete[] m_desired_speed;
    delete[] m_accel;
    delete[] m_current_speed;
    delete[] m_current_pos;
}
    
void VelocityController::initialize() {
    for (uint8_t i = 0; i < m_num_motors; i++) {
        m_control.clearTravelPosition(i);
        setSpeedTicks(i, 0);
        // maximum hw accel, using sw accel limiting
        m_control.setSpeedRampRate(i, 0xff);

        m_set_speed[i] = 0.0;
        m_current_pos[i] = 0;
        m_current_speed[i] = 0;
        m_accel[i] = DEFAULT_ACCEL_VALUE * TICKS_TO_RADIANS;
    }
    ROS_INFO("Initialized velocity controller");
    backgroundTask();
}

void VelocityController::backgroundTask() {
    for (uint8_t i = 0; i < m_num_motors; i++) {
        // query current encoder data
        m_current_pos[i] = m_control.queryPosition(i);
        m_current_speed[i] = m_control.queryAvgSpeed(i) * 2; // speed is per 0.5 second by default

        // calculate new set speeds
        // delta speed per time period according to acceleration
        double eff_accel = m_accel[i] / m_rate;

        // special case for zero
        if (m_desired_speed[i] == 0) {
            m_set_speed[i] = 0;
        // if relatively small change
        } else if (abs(m_desired_speed[i] - m_set_speed[i]) <= eff_accel) {
            m_set_speed[i] = m_desired_speed[i];
        // if larger change, clamp to eff_accel
        } else {
            if (m_desired_speed[i] > m_set_speed[i]) {
                m_set_speed[i] += eff_accel;
            } else {
                m_set_speed[i] -= eff_accel;
            }
        }

        // actually set the speeds
        m_control.setMaximumSpeed(i, static_cast<int16_t>(abs(m_set_speed[i] * RADIANS_TO_TICKS) / 2));
        // this is an attempt to synchronize the travel queue with calls to
        // backgroundTask. May not work as expected... TEST!!
        m_control.queueTravelPosition(i, static_cast<int16_t>(m_set_speed[i] * RADIANS_TO_TICKS / m_rate));
    }
}

void VelocityController::setSpeedTicks(uint8_t id, int16_t speed) {
    setSpeedRadians(id, speed * TICKS_TO_RADIANS);
}

void VelocityController::setSpeedRadians(uint8_t id, double speed) {
    m_desired_speed[id] = speed;
}

void VelocityController::setAccelTicks(uint8_t id, int16_t accel) {
    setAccelRadians(id, accel * TICKS_TO_RADIANS);
}

void VelocityController::setAccelRadians(uint8_t id, double accel) {
    m_accel[id] = accel;
}

double VelocityController::getAcceleration(uint8_t id) {
    return m_accel[id];
}

double VelocityController::getPositionRadians(uint8_t id) {
    return getPositionTicks(id) * TICKS_TO_RADIANS;
}

int16_t VelocityController::getPositionTicks(uint8_t id) {
    return m_current_pos[id];
}

double VelocityController::getSpeedRadians(uint8_t id) {
    return getSpeedTicks(id) * TICKS_TO_RADIANS;
}

int16_t VelocityController::getSpeedTicks(uint8_t id) {
    return m_current_speed[id];
}
