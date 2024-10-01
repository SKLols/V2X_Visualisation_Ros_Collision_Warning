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
#include "cam_provider.hpp"


CamReceiver::CamReceiver(ros::NodeHandle nh, ros::NodeHandle nh_private, ros::Publisher publisher):
    m_nh(nh), m_nh_private(nh_private), m_publisher(publisher){

    std::string cam_topic;
    //m_nh_private.getParam("cam_provider_topic", cam_topic);

    m_sub = nh.subscribe("cam_received", 1000, &CamReceiver::receiveCam, this);
}


void CamReceiver::receiveCam(const etsi_its_msgs::CAMPtr& cam) {
    map_manager::MapObject mapObject;
    
    mapObject.position = cam->reference_position; //In micro degree * 10 (from standard)
    mapObject.heading = cam->high_frequency_container.heading;

    switch (cam->station_type.value){
        case etsi_its_msgs::StationType::PASSENGER_CAR:
            mapObject.type = "VEHICLE";
            break ;
        case etsi_its_msgs::StationType::LIGHT_TRUCK:
            mapObject.type = "TRUCK";
            break;
        case etsi_its_msgs::StationType::HEAVY_TRUCK:
            mapObject.type = "TRUCK";
            break;
        case etsi_its_msgs::StationType::MOTORCYCLE:
            mapObject.type = "MOTORCYCLE";
            break;
        case etsi_its_msgs::StationType::PEDESTRIAN:
            mapObject.type = "PEDESTRIAN";
            break;
        case etsi_its_msgs::StationType::CYCLIST:
            mapObject.type = "CYCLIST";
            break;
        case etsi_its_msgs::StationType::ROAD_SIDE_UNIT:
            mapObject.type = "RSU";
            break;
        case etsi_its_msgs::StationType::MOPED:
            mapObject.type = "PDK";
            break;
        case etsi_its_msgs::StationType::SPECIAL_VEHICLE:
            mapObject.type = "Level0";
            break;
        case etsi_its_msgs::StationType::TRAM:
            mapObject.type = "Level1";
            break;
        case etsi_its_msgs::StationType::TRAILER:
            mapObject.type = "Level2";
            break;
        case etsi_its_msgs::StationType::BUS:
            mapObject.type = "Level3";
            break;            
        default:
            mapObject.type = "UNKNOWN";
    }

    mapObject.source = "CAM";
    mapObject.source_id = cam->its_header.station_id;

    if(m_cams_received.count(cam->its_header.station_id)){
        mapObject.id = m_cams_received[cam->its_header.station_id];
    } else {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        mapObject.id = boost::uuids::to_string(uuid);
        m_cams_received[cam->its_header.station_id] = mapObject.id;
    }

    mapObject.expiration_time = 1.3; //in second

    m_publisher.publish(mapObject);
}


