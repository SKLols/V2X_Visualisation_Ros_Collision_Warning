//
// Created by rosk on 30.08.19.
//

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <map_manager/MapObject.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <ublox_msgs/NavPVT.h>


class UbloxReceiver
{
public:
    UbloxReceiver(ros::NodeHandle nh, ros::Publisher publisher);
    void receiveGPS(const ublox_msgs::NavPVTPtr& gps_msg);

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_publisher;
    std::string m_id;
};
