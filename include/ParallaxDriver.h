#ifndef PARALLAX_POSITION_CONTROLLER_DRIVER_H_
#define PARALLAX_POSITION_CONTROLLER_DRIVER_H_ 

#include <string>
#include <serial/serial.h>

/**
 * Driver for Parallax Position Controller
 */
class ParallaxPositionController {
    public:
        /**
         * Constructs a ParallaxPositionController
         * @param ser the port to use
         * @param id a list of ids to use
         * @param num the number of ids in the list
         */
        ParallaxPositionController(std::string ser, uint8_t * id, uint8_t num);

        /**
         * Destructs the controller. Note, does NOT destruct the serial port
         */
        ~ParallaxPositionController();

        /**
         * Opens the serial port
         */
        void open();

        /**
         * Closes the serial port
         */
        void close();

        /**
         * Checks if the port is open
         * @return 0 if the port is not open
         */
        uint8_t isOpen();

        /**
         * Gets the port name
         * @return name of the current serial port
         */
        std::string getPortName();

        // ------------------ Standard API Functions ----------------------

        /**
         * Queries the current position of the wheel. 
         * @param id which position controller to query
         * @return position of the wheel in encoder ticks
         */
        int16_t queryPosition(uint8_t id);

        /**
         * Queries the average speed of the wheel over the past 0.5 sec
         * @param id which position controller to query
         * @return speed of the wheel in encoder ticks per 0.5 second
         */
        int16_t queryAvgSpeed(uint8_t id);

        /**
         * Checks if the wheel is at the requested position, given a 
         * particular tolerance
         * @param id which position controller to use
         * @param tolerance tolerance in encoder ticks
         * @return true if wheel is within tolerance ticks of its destination
         */
        uint8_t isAtPosition(uint8_t id, uint8_t tolerance);

        /**
         * Queues a new travel position, specified in encoder ticks
         * @param id which position controller to use
         * @param position the new relative position to travel to
         */
        void queueTravelPosition(uint8_t id, int16_t position);

        /**
         * Resets the current position, end point, and set point back to zero.
         * Will effectively halt the wheel at its current position.
         * Equivalent to a soft-reset of the position controller.
         * @param id which position controller to query
         */
        void clearTravelPosition(uint8_t id);

        /**
         * Reverses the orientation of this particular position controller.
         * Can only be cleared by power-cycling the controller
         * @param id which position controller to use
         */
        void setPositionAsReversed(uint8_t id);

        /**
         * Sets the minimum amount of time the controller will wait before
         * sending data back on the half-duplex serial line.
         * @param id which position controller to use
         * @param seconds the length of the delay in seconds
         */
        void setMinimumDelayValue(uint8_t id, float seconds);

        /**
         * Sets the maximum speed of the wheel in ticks per 0.5 sec
         * @param id which position controller to use
         * @param speed speed in ticks per second
         */
        void setMaximumSpeed(uint8_t id, uint16_t speed);

        /**
         * Sets the maximum acceleration in ticks per 0.25 sec squared
         * @param id which position controller to use
         * @param accel acceleration in ticks per 0.25 sec squared
         */
        void setSpeedRampRate(uint8_t id, uint8_t accel);

        // ---------------- End Standard API Functions --------------------
        
        // ------------------- Custom API Functions -----------------------

        // ----------------- End Custom API Functions ---------------------

    private:
        /**
         * Initializes the position controller
         */
        void initialize();

        /**
         * Conducts a serial transaction
         * @param id the id to send to (0 is broadcast)
         * @param cmd the command to send, from PPCCommand::command_t
         * @param sendbuf the buffer to send from
         * @param readbuf the buffer to read bytes into
         * @return the number of bytes read
         */
        size_t transaction(uint8_t id, uint8_t cmd, uint8_t * sendbuf, uint8_t * readbuf);

        /**
         * Deserializes a 16-bit signed integer. Assumes that the high byte is
         * first.
         * @param buf the buffer to read from. Precondition: buf holds at least
         * two bytes
         * @return a 16-bit signed integer formed by shifting the bytes in buf
         */
        int16_t construct_int16(uint8_t * buf);

        /**
         * Allocates a new string to hold all members of the byte array,
         * printed in the format 0xXX where XX is the hexadecimal value of the
         * byte.
         * @param buf the byte array to print
         * @param buflen the length of the byte array
         * @return pointer to string (needs to be freed
         */
        char* byteArrayToString(uint8_t * buf, uint8_t buflen);

        // members
        std::string m_portname;
        serial::Serial * m_serial;
        uint8_t* m_id, m_id_count;

        // constants
        static const int BAUDRATE = 19200;
        static const int TIMEOUT = 100;

};

#endif /* PARALLAX_POSITION_CONTROLLER_DRIVER_H_ */
