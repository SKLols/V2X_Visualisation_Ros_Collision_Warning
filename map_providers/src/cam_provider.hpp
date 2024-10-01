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
#include <etsi_its_msgs/CAM.h>

class CamReceiver
{
    public:
        CamReceiver(ros::NodeHandle nh, ros::NodeHandle nh_private, ros::Publisher publisher);
        void receiveCam(const etsi_its_msgs::CAMPtr& cam);

    private:
        ros::NodeHandle m_nh;
        ros::NodeHandle m_nh_private;
        ros::Subscriber m_sub;
        ros::Publisher m_publisher;
        std::map<uint32_t, std::string> m_cams_received;
};
