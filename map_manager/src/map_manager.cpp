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
#include <map_manager/ListMapObjects.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <ros/console.h>

std::map<std::string, map_manager::MapObject> map_objects;

void receiveObjects(const map_manager::MapObjectPtr& msg)
{
    map_objects[msg->id] = *msg; // In case we have more than one update for the same object
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int rate;
    nh_private.getParam("updating_frequency", rate);
    ros::Rate r(rate);

    ros::Publisher publisher_map_objects = nh.advertise<map_manager::ListMapObjects>("map_objects", 1000);
    ros::Subscriber sub = nh.subscribe("map_manager", 1000, receiveObjects);

    map_manager::ListMapObjects list_map_object;
    std::vector<map_manager::MapObject> list_objects;
    map_manager::MapObject mapObject;

    mapObject.position.latitude = 48.765957 * 1000000; //In micro degree
    mapObject.position.longitude = 11.434670 * 1000000;

    mapObject.heading.value = 2300;
    mapObject.type = "VEHICLE";
    mapObject.source = "GPS";

    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    mapObject.id = boost::uuids::to_string(uuid);

    mapObject.expiration_time = 1; //in second

    list_objects.push_back(mapObject);

    list_map_object.list_objects = list_objects;

    while(ros::ok()){

        //TODO put inside the loop
        //publisher_map_objects.publish(list_map_object);
        ros::spinOnce();

        if(!map_objects.empty()){

            for(auto obj : map_objects)
                list_objects.push_back(obj.second);

            list_map_object.list_objects = list_objects;

            publisher_map_objects.publish(list_map_object);

            // No need to keep the objects already transmitted once
            map_objects.clear();
            list_objects.clear();
        }

        r.sleep();
    }
}
