//
// Created by delooz on 05.03.20.
//

#include "objects_provider.hpp"


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "etsi_its_msgs/PerceivedObject.h"
#include "etsi_its_msgs/ListOfPerceivedObjects.h"
#include "ublox_msgs/NavPVT.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <math.h>
#include "map_providers/MapObject.h"



# define M_PIl          3.141592653589793238462643383279502884L /* pi */

using namespace std;


double R = 6371000; //Radius of the Eartth in meters


ObjectsReceiver::ObjectsReceiver(ros::NodeHandle nh, ros::Publisher publisher): m_nh(nh), m_publisher(publisher){
    m_sub_lo = m_nh.subscribe("/sensor_lo", 1000, &ObjectsReceiver::geocoordinate, this);
    m_sub_gps = m_nh.subscribe("/gnss/navpvt", 1000,  &ObjectsReceiver::process_gps, this);
    //m_sub_imu = m_nh.subscribe("/vmu931/data", 1000, process_imu);
}


void ObjectsReceiver::process_imu(const sensor_msgs::Imu::ConstPtr& imu)
{
    double w = imu->orientation.w;
    double x = imu->orientation.x;
    double y = imu->orientation.y;
    double z = imu->orientation.z;

    /*Pitch convertion from quaternion to Euler*/
    m_vehiclePitch = asin(2 * (w*y - z*x));

    /*define Yaw angle from quartenion*/
    double siny_cosp = 2*(w*z +x*y);
    double cosy_cosp = 1 - 2*(y*y + z*z);
    m_vehicleYaw = atan2(siny_cosp, cosy_cosp);
    m_vehicleYawDegree = m_vehicleYaw*(180/M_PIl);

}

void ObjectsReceiver::process_gps(const ublox_msgs::NavPVT::ConstPtr& gps)
{
    m_vehicleLat = gps->lat;
    m_vehicleLon = gps->lon;
    m_vehicleHeading = gps->headVeh;
    m_vehicleHeight = gps->height;
    m_headOfMotion = gps->heading;
}

void ObjectsReceiver::geocoordinate(const etsi_its_msgs::ListOfPerceivedObjects& objectList)
{
    std::vector<etsi_its_msgs::PerceivedObject> vectorObject;
    vectorObject = objectList.perceivedObjectContainer;
    etsi_its_msgs::PerceivedObject objectDetected;
    std::vector<etsi_its_msgs::PerceivedObject>::iterator obj;

    //Match GPS point to Mobileye reference point
    double refPointLat;
    double refPointLon;

    float GPSrefPointToVehicleCenter = 0.45; //in meter
    float GPSrefPointToVehicleFront = 3.24; //in meter

    float distanceGPStoRefPoint = sqrt((GPSrefPointToVehicleCenter)*(GPSrefPointToVehicleCenter) + (GPSrefPointToVehicleFront)*(GPSrefPointToVehicleFront)); //Euclidean distance between GPS position and Ref Point
    float angleGPSandRefPoint = atan(GPSrefPointToVehicleCenter/GPSrefPointToVehicleFront); //Internal angle between GPS and Ref Point
    float distanceGPStoRefPointGeo = distanceGPStoRefPoint*abs(cos(m_vehiclePitch)); //Correction for pitch angles

    double GPSLatDegree = m_vehicleLat/10000000; //convert the lat string to degree
    double GPSLonDegree = m_vehicleLon/10000000; //convert the long string to degree
    double vehicleHeadingDegree = m_vehicleHeading/100000; //convert the heading string to degree

    double GPSLatRad = GPSLatDegree*M_PIl/180;
    double GPSLonRad = GPSLonDegree*M_PIl/180;
    double vehicleHeadingRad = vehicleHeadingDegree*M_PIl/180; //convert all values to RADIANS

    refPointLat = asin(sin(GPSLatRad)*cos(distanceGPStoRefPointGeo/R) + cos(GPSLatRad)*sin(distanceGPStoRefPointGeo/R)*cos(vehicleHeadingRad)); //calculate the Ref Point Latitude
    refPointLon = GPSLonRad + atan2(sin(vehicleHeadingRad)*sin(distanceGPStoRefPointGeo/R)*cos(GPSLatRad),cos(distanceGPStoRefPointGeo/R) - sin(GPSLatRad)*sin(refPointLat)); //calculate the Ref Point Longitude

    for(obj = vectorObject.begin() ; obj != vectorObject.end() ; obj++)
    {

        objectDetected.objectID = obj->objectID;
        objectDetected.xDistance.value = obj->xDistance.value;
        objectDetected.yDistance.value = obj->yDistance.value;
        objectDetected.objectAngle = obj->objectAngle;
        objectDetected.classification.value = obj->classification.value;

        //Object Azimuth definition
        double objectAziDegree = (vehicleHeadingDegree + objectDetected.objectAngle);

        //Convert Degree to Rad
        double objectVehicleAngleRad = objectDetected.objectAngle*M_PIl/180; //angle between vehicle and object in Rad

        //Calculate the distance between object and vehicle
        double objectEuclideanDistance = objectDetected.xDistance.value/abs(cos(objectVehicleAngleRad));

        //Calculate Object lat and long
        double objectLatRad;
        double objectLonRad;
        double objectLatDegree;
        double objectLonDegree;
        double objectAziRad;

        if (objectAziDegree < 0)
        {
            objectAziRad = (360 + objectAziDegree)*M_PIl/180;
        }

        if (objectAziDegree > 360)
        {
            objectAziRad =(objectAziDegree - 360)*M_PIl/180;
        }

        else
        {
            objectAziRad = objectAziDegree*M_PIl/180;
        }

        //correct the surface inclination
        double objectGeographicalDistance = objectEuclideanDistance*abs(cos(m_vehiclePitch));

        //getting Object Lat and Long
        objectLatRad = asin(sin(refPointLat)*cos(objectGeographicalDistance/R) + cos(refPointLat)*sin(objectGeographicalDistance/R)*cos(vehicleHeadingRad));
        objectLonRad = refPointLon + atan2(sin(vehicleHeadingRad)*sin(objectGeographicalDistance/R)*cos(refPointLat),cos(objectGeographicalDistance/R) - sin(refPointLat)*sin(objectLatRad));

        objectLatDegree = objectLatRad*(180/M_PIl);
        objectLonDegree = objectLonRad*(180/M_PIl);

        //Measure the error
        double aDistance;
        double cDistance;
        double dDistance;
        double diffe;

        aDistance = sin((objectLatRad - refPointLat)/2)*sin((objectLatRad - refPointLat)/2) + cos(refPointLat)*cos(objectLatRad)*sin((objectLonRad - refPointLon)/2)*sin((objectLonRad - refPointLon)/2);
        cDistance = 2*atan2(sqrt(aDistance), sqrt(1-aDistance));
        dDistance = R*cDistance;
        diffe = abs(dDistance - objectGeographicalDistance);

        std::cout.precision(20);

/*                std::cout << "ObjectID: " << objectDetected.objectID << std::endl;
                std::cout << "Euclidiana: " << objectEuclideanDistance << std::endl;
                std::cout << "Camera Distance " << objectGeographicalDistance << std::endl;
                std::cout << "GeoDistance " << dDistance << std::endl;
                std::cout << "Error Geo: " << diffe << std::endl;
		std::cout << "OBJECT "  << std::endl;
		std::cout << "latitude: " << objectLatDegree << std::endl;
		std::cout << "longitude: " << objectLonDegree << '\n' << std::endl;
                std::cout << "VEHICLE" << std::endl;
                std::cout << "latitude: " << GPSLatDegree << std::endl;
		std::cout << "longitude: " << GPSLonDegree << '\n' << std::endl;
                std::cout << "REF. POINT" << std::endl;
                std::cout << "latitude: " << refPointLat*(180/M_PIl) << std::endl;
                std::cout << "longitude: " << refPointLon*(180/M_PIl) << '\n' << std::endl;*/
        /*std::cout << "GPS" << std::endl;
        std::cout << "vehicleHeading: " << vehicleHeadingDegree << std::endl;
        std::cout << "Heading of Motion: " << m_headOfMotion << std::endl;
        std::cout << "IMU" << std::endl;
        std::cout << "Yaw: " << m_vehicleYawDegree << std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;
    */
        //Publishing
        map_providers::MapObject objectGeocoordinate;

        ros::Time timeNow = ros::Time::now();
        objectGeocoordinate.header.stamp = timeNow;
        objectGeocoordinate.header.frame_id = "mobileye";
        objectGeocoordinate.position.latitude = objectLatDegree * 10000000;
        objectGeocoordinate.position.longitude = objectLonDegree * 10000000;
        objectGeocoordinate.heading.value = vehicleHeadingDegree*10;

        /*calculate object hight*/
        double objectHeight;
        if (m_vehiclePitch < 0) objectHeight = m_vehicleHeight - cos(m_vehiclePitch)*objectEuclideanDistance;
        if (m_vehiclePitch > 0) objectHeight = m_vehicleHeight + cos(m_vehiclePitch)*objectEuclideanDistance;
        if (m_vehiclePitch = 0) objectHeight = m_vehicleHeight;
        objectGeocoordinate.position.altitude.value = round(objectHeight*0.1);

        switch (objectDetected.classification.value){
            case 0:
                objectGeocoordinate.type = "VEHICLE";
                break;
            case 1:
                objectGeocoordinate.type = "TRUCK";
                break;
            case 2:
                objectGeocoordinate.type = "MOTORCYCLE";
                break;
            case 3:
                objectGeocoordinate.type = "PEDESTRIAN";
                break;
            case 4:
                objectGeocoordinate.type = "CYCLIST";
                break;
            case 5:
                objectGeocoordinate.type = "Collision";
                break;    
            default:
                objectGeocoordinate.type = "UNKNOWN";
        }

        objectGeocoordinate.source = "MOBILEYE";

        objectGeocoordinate.expiration_time = 0.26; //s

        objectGeocoordinate.source_id = obj->objectID;

        if(m_objects_received.count(obj->objectID)){
            objectGeocoordinate.id = m_objects_received[obj->objectID];
        } else {
            boost::uuids::uuid uuid = boost::uuids::random_generator()();
            objectGeocoordinate.id = boost::uuids::to_string(uuid);
            m_objects_received[obj->objectID] = objectGeocoordinate.id;
        }

        m_publisher.publish(objectGeocoordinate);
    }
}
