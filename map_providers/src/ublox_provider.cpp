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
#include "ublox_provider.hpp"


UbloxReceiver::UbloxReceiver(ros::NodeHandle nh, ros::Publisher publisher): m_nh(nh), m_publisher(publisher){

    m_sub = nh.subscribe("gnss/navpvt", 1000, &UbloxReceiver::receiveGPS, this);
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    m_id = boost::uuids::to_string(uuid);
}


void UbloxReceiver::receiveGPS(const ublox_msgs::NavPVTPtr& gps_msg) {
    map_manager::MapObject mapObject;

    mapObject.position.latitude = gps_msg->lat; //In micro degree * factor standard
    mapObject.position.longitude = gps_msg->lon; //In micro degree * factor standard
    mapObject.heading.value = gps_msg->headVeh / 10000.0;
    mapObject.heading.confidence = gps_msg->headAcc / 10000.0;

    mapObject.type = "VEHICLE";
    mapObject.source = "GPS";
    mapObject.id = m_id;

    mapObject.expiration_time = 1; //in second

    m_publisher.publish(mapObject);
}


