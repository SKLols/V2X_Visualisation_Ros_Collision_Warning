//
// Created by delooz on 05.03.20.
//

#ifndef MAP_LOCALISATION_SERVER_MOBILEYE_OBJECTS_H
#define MAP_LOCALISATION_SERVER_MOBILEYE_OBJECTS_H

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
#include "etsi_its_msgs/PerceivedObject.h"
#include "etsi_its_msgs/ListOfPerceivedObjects.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <string>
#include <math.h>
#include "map_providers/MapObject.h"


class ObjectsReceiver
{
public:
    ObjectsReceiver(ros::NodeHandle nh, ros::Publisher publisher);
    void process_imu(const sensor_msgs::Imu::ConstPtr& imu);
    void process_gps(const ublox_msgs::NavPVT::ConstPtr& gps);
    void geocoordinate(const etsi_its_msgs::ListOfPerceivedObjects& objectList);

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_publisher;
    ros::Subscriber m_sub_lo;
    ros::Subscriber m_sub_gps;
    ros::Subscriber sub_imu;
    std::string m_id;

    double m_vehiclePitch;
    double m_vehicleLat;
    double m_vehicleLon;
    double m_vehicleHeading;
    double m_vehiclePitchAngle;
    double m_vehicleYaw;
    double m_vehicleYawDegree;
    float m_vehicleHeight;
    double m_headOfMotion;

    std::map<uint8_t, std::string> m_objects_received;
};



#endif //MAP_LOCALISATION_SERVER_MOBILEYE_OBJECTS_H
