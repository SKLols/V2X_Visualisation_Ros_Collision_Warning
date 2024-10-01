//
// Created by delooz on 05.03.20.
//

#ifndef MAP_CPM_PROVIDER_H
#define MAP_CPM_PROVIDER_H

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <random>
#include <map_manager/MapObject.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "etsi_its_msgs/PerceivedObject.h"
#include "etsi_its_msgs/ListOfPerceivedObjects.h"
#include "etsi_its_msgs/CPM.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <string>
#include <math.h>
#include "map_providers/MapObject.h"


class CPMReceiver
{
    public:
        CPMReceiver(ros::NodeHandle nh, ros::Publisher publisher);
        void processCPM(const etsi_its_msgs::CPMPtr& cpm);
        void generateMapMarker(double objectLatDegree, double objectLonDegree, double vehicleHeadingDegree,
                uint8_t type, std::string source, uint32_t stationId, uint32_t objectId);
    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_publisher;
        ros::Subscriber m_sub_cpm;
        std::string m_id;
        std::map<uint32_t, std::string> m_cpms_received;
        const double m_radiusEarth = 6371000;
};



#endif //MAP_CPM_PROVIDER_H
