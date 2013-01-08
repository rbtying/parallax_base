#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_

#include "ParallaxDriver.h"
#include <ros/ros.h>

class VelocityController {
    public:
        /**
         * Constructs a VelocityController object with given parameters
         * @param portname the name of the port
         * @param ids list of motor ids
         * @param idcount number of motor ids
         * @param rate the update rate at which the background task will be
         * called
         */
        VelocityController(std::string portname, uint8_t * ids, uint8_t idcount, double rate);

        /**
         * Destructs the velocitycontroller
         */
        ~VelocityController();

        /**
         * Gets the speed of the given motor
         * @param id the motor to use
         * @return speed in ticks per second
         */
        int16_t getSpeedTicks(uint8_t id);

        /**
         * Gets the speed of the given motor
         * @param id the motor to use
         * @return speed in radians per second
         */
        double getSpeedRadians(uint8_t id);

        /**
         * Gets the position of the given motor
         * @param id the motor to use
         * @return the position in ticks
         */
        int16_t getPositionTicks(uint8_t id);

        /**
         * Gets the position of the given motor
         * @param id the motor to use
         * @return the position in radians
         */
        double getPositionRadians(uint8_t id);

        /**
         * Gets the max acceleration of the given motor
         * @param id the motor to use
         * @return the acceleration in rad/s^2
         */
        double getAcceleration(uint8_t id);

        /**
         * Sets the desired speed
         * @param id the motor to use
         * @param speed the speed in ticks per second
         */
        void setSpeedTicks(uint8_t id, int16_t speed);

        /**
         * Sets the desired speed
         * @param id the motor to use
         * @param speed the speed in radians per second
         */
        void setSpeedRadians(uint8_t id, double speed);

        /**
         * Sets the desired acceleration
         * @param id the motor to use
         * @param accel the acceleration in ticks per second squared
         */
        void setAccelTicks(uint8_t id, int16_t accel);

        /**
         * Sets the desired acceleration
         * @param id the motor to use
         * @param accel the acceleration in radians per second squared
         */
        void setAccelRadians(uint8_t id, double accel);

        /**
         * Worker task that actually does the communications. Should be
         * called at the rate given in the constructor; bad things will
         * happen if it isn't.
         */
        void backgroundTask();

    private:
        ParallaxPositionController m_control;

        double m_rate;

        uint8_t m_num_motors;
        
        double * m_set_speed;
        double * m_desired_speed;
        double * m_accel;
        int16_t * m_current_speed;
        int16_t * m_current_pos;

        static const int16_t MAX_VALUE_INT16 = 32767;
        // original default acceleration value is 15 ticks/0.25 sec^2
        static const int DEFAULT_ACCEL_VALUE = 15 * 4;

        /**
         * Initializes the motors
         */
        void initialize();

        static const double TICKS_PER_ROTATION;
        static const double PI;
        static const double TICKS_TO_RADIANS;
        static const double RADIANS_TO_TICKS;
};

#endif /* VELOCITY_CONTROLLER_H_ */
