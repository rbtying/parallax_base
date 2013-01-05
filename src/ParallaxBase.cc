#include "ParallaxBase.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <parallax_base/RawData.h>

ParallaxBase::ParallaxBase() : m_n("~") {
    // get parameters
    m_n.param<std::string>("nodename", m_nodename, "parallax_base");
    m_n.param<std::string>("framename", m_framename, "parallax_base_frame");

    m_n.param<double>("max_linear_speed", m_max_linear_speed, 0.0);
    m_n.param<double>("max_rotational_speed", m_max_rotational_speed, 0.0);

    // set up publishers
    m_odometry_publisher = m_n.advertise<nav_msgs::Odometry>(m_nodename + "/wheel_odom", 1);
    m_auxiliary_data_publisher = m_n.advertise<parallax_base::RawData>(m_nodename + "/aux_data", 1);

    // set up subscribers
    m_twist_subscriber = m_n.subscribe<geometry_msgs::Twist>(m_nodename + "/cmd_vel",
            10, &ParallaxBase::twistCallback, this);

    // initialize variables
    m_seq = 0;
}

void ParallaxBase::twistCallback(const geometry_msgs::Twist::ConstPtr& twist) {

}

void ParallaxBase::fillHeader(std_msgs::Header * header) {
    header->seq = m_seq;
    header->stamp = ros::Time::now();
    header->frame_id = m_framename;
}

void ParallaxBase::publishData() {
    fillHeader(&m_rawdata.header);
    m_auxiliary_data_publisher.publish(m_rawdata);
}
