//
// Created by delooz on 05.03.20.
// Adapted from Silas Lobo work
//

#include "cpm_provider.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "etsi_its_msgs/PerceivedObject.h"
#include "etsi_its_msgs/ListOfPerceivedObjects.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include "map_providers/MapObject.h"



# define M_PIl          3.141592653589793238462643383279502884L /* pi */

using namespace std;


CPMReceiver::CPMReceiver(ros::NodeHandle nh, ros::Publisher publisher): m_nh(nh), m_publisher(publisher){
    m_sub_cpm = m_nh.subscribe("cpm_received", 1000, &CPMReceiver::processCPM, this);
}


void CPMReceiver::processCPM(const etsi_its_msgs::CPMPtr& cpm)
{
    std::vector<etsi_its_msgs::PerceivedObject> vectorObject;
    vectorObject = cpm->listOfPerceivedObjects.perceivedObjectContainer;
    etsi_its_msgs::PerceivedObject objectDetected;
    std::vector<etsi_its_msgs::PerceivedObject>::iterator obj;

    //Match GPS point to Mobileye reference point
    double refPointLat = cpm->reference_position.latitude / 10000000.0; //Convert micro to degree
    double refPointLon = cpm->reference_position.longitude / 10000000.0; //Convert micro to degree
    double vehicleHeadingDegree = cpm->originatingVehicleContainer.heading.value / 10.0; //From decidegree to degree
    double vehicleHeadingRad = vehicleHeadingDegree*M_PIl/180; //convert all values to RADIANS

    generateMapMarker(refPointLat, refPointLon, vehicleHeadingDegree, cpm->station_type.value, "CPM-S", cpm->its_header.station_id, cpm->its_header.station_id);

    refPointLat = refPointLat * M_PIl/180; //Convert degree to radian
    refPointLon = refPointLon * M_PIl/180; //Convert degree to radian


    for(obj = vectorObject.begin() ; obj != vectorObject.end() ; obj++) {

        objectDetected.objectID = obj->objectID;
        objectDetected.xDistance.value = obj->xDistance.value;
        objectDetected.yDistance.value = obj->yDistance.value;


        /* Important note: there are two different systems going on: heading is relatively to the north
         * Object angle is relatively to the vehicle, need to perform a conversion
         * */
        objectDetected.objectAngle = atan2(objectDetected.yDistance.value, objectDetected.xDistance.value) * 180 / M_PIl;
        //std::cout << "station " << cpm->its_header.station_id <<  " x " << objectDetected.xDistance.value << "  y: " <<   obj->yDistance.value << " angle " << objectDetected.objectAngle << " heading " << vehicleHeadingDegree <<  std::endl;


        objectDetected.classification.value = obj->classification.value;

        /* Important note: conversion happens here! (take -)
            * */
        double objectAziDegree = vehicleHeadingDegree - objectDetected.objectAngle;

        //Convert Degree to Rad
        double objectVehicleAngleRad =
                objectDetected.objectAngle * M_PIl / 180; //angle between vehicle and object in Rad

        //Calculate the distance between object and vehicle
        double objectEuclideanDistance = sqrt(pow(objectDetected.xDistance.value, 2) + pow(objectDetected.yDistance.value, 2));//  / objectDetected.xDistance.value * abs(cos(objectVehicleAngleRad));

        //Calculate Object lat and long
        double objectLatRad;
        double objectLonRad;
        double objectLatDegree;
        double objectLonDegree;
        double objectAziRad;

        if (objectAziDegree < 0) {
            objectAziRad = (360 + objectAziDegree) * M_PIl / 180;
        }

        if (objectAziDegree > 360) {
            objectAziRad = (objectAziDegree - 360) * M_PIl / 180;
        } else {
            objectAziRad = objectAziDegree * M_PIl / 180;
        }

        //correct the surface inclination
        double objectGeographicalDistance = objectEuclideanDistance * abs(cos(0)); //Take into consideration pitch angle if available.

        //getting Object Lat and Long
        objectLatRad = asin(sin(refPointLat) * cos(objectGeographicalDistance / m_radiusEarth) +
                            cos(refPointLat) * sin(objectGeographicalDistance / m_radiusEarth) *
                            cos(objectAziRad));
        objectLonRad = refPointLon + atan2(sin(objectAziRad) * sin(objectGeographicalDistance / m_radiusEarth) *
                                           cos(refPointLat), cos(objectGeographicalDistance / m_radiusEarth) -
                                                             sin(refPointLat) * sin(objectLatRad));

        objectLatDegree = objectLatRad * (180 / M_PIl);
        objectLonDegree = objectLonRad * (180 / M_PIl);

        //Measure the error
        double aDistance;
        double cDistance;
        double dDistance;
        double diffe;

        aDistance = sin((objectLatRad - refPointLat) / 2) * sin((objectLatRad - refPointLat) / 2) +
                    cos(refPointLat) * cos(objectLatRad) * sin((objectLonRad - refPointLon) / 2) *
                    sin((objectLonRad - refPointLon) / 2);
        cDistance = 2 * atan2(sqrt(aDistance), sqrt(1 - aDistance));
        dDistance = m_radiusEarth * cDistance;
        diffe = abs(dDistance - objectGeographicalDistance);

        generateMapMarker(objectLatDegree, objectLonDegree, vehicleHeadingDegree, objectDetected.classification.value, "CPM-O", cpm->its_header.station_id, objectDetected.objectID);
    }
}


void CPMReceiver::generateMapMarker(double objectLatDegree, double objectLonDegree, double vehicleHeadingDegree,
        uint8_t type, std::string source, uint32_t stationId, uint32_t objectId){
    //Publishing
    map_providers::MapObject marker;

    ros::Time timeNow = ros::Time::now();
    marker.header.stamp = timeNow;
    marker.header.frame_id = "cpm";
    marker.position.latitude = objectLatDegree * 10000000;
    marker.position.longitude = objectLonDegree * 10000000;
    marker.heading.value = vehicleHeadingDegree*10;

    //define object type
    switch (type){
        case etsi_its_msgs::StationType::PASSENGER_CAR:
            marker.type = "VEHICLE";
            break ;
        case etsi_its_msgs::StationType::LIGHT_TRUCK:
            marker.type = "TRUCK";
            break;
        case etsi_its_msgs::StationType::HEAVY_TRUCK:
            marker.type = "TRUCK";
            break;
        case etsi_its_msgs::StationType::MOTORCYCLE:
            marker.type = "MOTORCYCLE";
            break;
        case etsi_its_msgs::StationType::PEDESTRIAN:
            marker.type = "PEDESTRIAN";
            break;
        case etsi_its_msgs::StationType::CYCLIST:
            marker.type = "CYCLIST";
            break;
        case etsi_its_msgs::StationType::MOPED:
            marker.type = "PDK";
            break;            
        case etsi_its_msgs::StationType::SPECIAL_VEHICLE:
            marker.type = "Level0";
            break;
        case etsi_its_msgs::StationType::TRAM:
            marker.type = "Level1";
            break;
        case etsi_its_msgs::StationType::TRAILER:
            marker.type = "Level2";
            break;
        case etsi_its_msgs::StationType::BUS:
            marker.type = "Level3";
            break;                            
        default:
            marker.type = "UNKNOWN";
    }

    marker.source = source;
    marker.source_id = stationId;

    marker.expiration_time = 1.1;

    if(m_cpms_received.count(stationId + objectId)){
        marker.id = m_cpms_received[stationId + objectId];
    } else {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        marker.id = boost::uuids::to_string(uuid);
        m_cpms_received[stationId + objectId] = marker.id;
    }

    m_publisher.publish(marker);

}
