#ifndef PARALLAX_BASE_H_
#define PARALLAX_BASE_H_

// libraries
#include <string>

// ros
#include <ros/ros.h>

// messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <parallax_base/RawData.h>

/**
 * Class to implement a ParallaxBase node, which uses the Parallax Position
 * Controller driver to make a differential-drive robot
 */
class ParallaxBase {
    public:
        /**
         * Constructs a ParallaxBase node
         */
        ParallaxBase();
    private:
        /**
         * Callback function for twist message processing
         * @param twist the message to process
         */
        void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);

        /**
         * Fills the header with the current timestamp, frame, and seq
         * @param header a pointer to the header to fill
         */
        void fillHeader(std_msgs::Header * header);

        /**
         * Publishes current data from the Position Controllers
         */
        void publishData();

        // ROS handle
        ros::NodeHandle m_n;

        // Publishers
        ros::Publisher m_odometry_publisher;
        ros::Publisher m_auxiliary_data_publisher;

        // Subscribers
        ros::Subscriber m_twist_subscriber;

        // Parameters
        std::string m_nodename;
        std::string m_framename;
        double m_max_linear_speed;
        double m_max_rotational_speed;

        // Other members
        nav_msgs::Odometry m_odom;
        parallax_base::RawData m_rawdata;
        uint32_t m_seq;
};

#endif /* PARALLAX_BASE_H_ */
