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
#include "denm_provider.hpp"

//rostopic pub -r 1 /denm_received etsi_its_msgs::DENM "{management: {action_id: {station_id: 8, sequence_number: 1}, termination: 1, event_position: {latitude: 48.784322, latitude: 11.473343}, relevance_distance: {value: 1}, validity_duration: 5, station_type: {value: 5}}}"

DenmReceiver::DenmReceiver(ros::NodeHandle nh, ros::Publisher publisher):
    m_nh(nh), m_publisher(publisher){

    std::string denm_topic;

    m_sub = nh.subscribe("denm_received", 1000, &DenmReceiver::receiveDenm, this);
}


void DenmReceiver::receiveDenm(const etsi_its_msgs::DENMPtr& denm) {
    map_manager::MapObject mapObject;

    mapObject.position = denm->management.event_position; //In micro degree * 10 (from standard)

    switch (denm->management.station_type.value){
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

    mapObject.source = "DENM";
    mapObject.source_id = denm->management.action_id.station_id;

    if(!denm->management.termination){
        mapObject.expiration_time = 0.0; //in second
    } else {
        mapObject.expiration_time = denm->management.validity_duration;
    }

    if(m_denms_received.count(denm->management.action_id.station_id)){
        mapObject.id = m_denms_received[denm->management.action_id.station_id];
    } else {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        mapObject.id = boost::uuids::to_string(uuid);
        m_denms_received[denm->management.action_id.station_id] = mapObject.id;
    }

    switch (denm->management.relevance_distance.value){
        case 0:
            mapObject.radius_denm = 50;
        break ;
        case 1:
            mapObject.radius_denm = 100;
        break;
        case 2:
            mapObject.radius_denm = 200;
        break;
        case 3:
            mapObject.radius_denm = 500;
        break;
        case 4:
            mapObject.radius_denm = 1000;
        break;
        case 5:
            mapObject.radius_denm = 50000;
        break;
        case 6:
            mapObject.radius_denm = 10000;
            break;
        case 7:
            mapObject.radius_denm = 10001;
            break;
        default:
            mapObject.radius_denm = 0;
    }

    m_publisher.publish(mapObject);
}


