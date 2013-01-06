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
         * @param serial the port to use
         */
        ParallaxPositionController(serial::Serial * ser, uint8_t id);

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

        /**
         * Gets a pointer to the serial port object in use
         * @return pointer to serial object
         */
        serial::Serial * getSerialPort();

    private:
        /**
         * Resets all internal tracking values
         */
        void reset();

        /**
         * Initializes the position controller
         */
        void initialize();

        /**
         * Conducts a serial transaction
         * @param id the id to send to (0 is broadcast)
         * @param cmd the command to send
         * @param numToSend the number of data bytes to send
         * @param sendbuf the buffer to send from
         * @param numToRead the number of bytes to expect to read back
         * @param readbuf the buffer to read bytes into
         * @return the number of bytes read
         */
        size_t transaction(uint8_t id, uint8_t cmd, uint8_t numToSend, 
                uint8_t * sendbuf, uint8_t numToRead, uint8_t * readbuf);

        // members
        std::string m_portname;
        serial::Serial * m_serial;
        uint8_t m_id;

        // constants
        static const int BAUDRATE = 19200;
        static const int TIMEOUT = 100;
};

#endif /* PARALLAX_POSITION_CONTROLLER_DRIVER_H_ */
