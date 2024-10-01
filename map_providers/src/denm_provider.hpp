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
#include <etsi_its_msgs/DENM.h>

class DenmReceiver
{
    public:
        DenmReceiver(ros::NodeHandle nh, ros::Publisher publisher);
        void receiveDenm(const etsi_its_msgs::DENMPtr& denm);

    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_sub;
        ros::Publisher m_publisher;
        std::map<uint32_t, std::string> m_denms_received;
};
