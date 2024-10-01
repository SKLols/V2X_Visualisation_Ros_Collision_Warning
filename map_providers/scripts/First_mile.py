#!/usr/bin/env python

import rospy
from etsi_its_msgs.msg import PerceivedObject, ListOfPerceivedObjects, CPM
from first_mile_msgs.msg import TrackObjectList, TrackObject
import math
from math import *
import random

# Define a function to process CPM messages and convert them to First Mile format
def First_mile_CPM (data):
    FirstmileObject=TrackObjectList()
    
    # Extract relevant information from the CPM message
    Header=data.header
    ItsPduHeader=data.its_header
    Stationid=ItsPduHeader.station_id
    DeltaTime=data.generation_delta_time
    StationType=data.station_type.value
    ReferencePosition=data.reference_position
    OriginatingVehicleContainer=data.originatingVehicleContainer
    Speed=OriginatingVehicleContainer.speed.value
    WGS84Angle=OriginatingVehicleContainer.vehicleOrientationAngle.value
    SensorInformationContainer=data.sensorInformationContainer
    ListOfPerceivedObjects=data.listOfPerceivedObjects
    number_of_obj=data.numberOfPerceivedObjects
    PerceivedObject=ListOfPerceivedObjects.perceivedObjectContainer

    # Populate the First Mile object with relevant information
    FirstmileObject.header=data.header
    FirstmileObject.frame_id=random.randint(0,20)
    FirstmileObject.mast_id='RSU_13'
    FirstmileObject.sensor_id='V2X_CPM'
    FirstmileObject.station_id=str(Stationid)
    FirstmileObject.total_objects=number_of_obj
    FirstmileObject.info=str(StationType)
    
    # Process each perceived object in the ListOfPerceivedObjects
    for x in PerceivedObject:
        ObjectID=x.objectID
        #for y in PerceivedObject:
        global pose_x, pose_y, speed_x, speed_y, speed_z, acceleration_x,pose_z, acceleration_y
        
        # Extract information about the perceived object
        pose_x=x.xDistance.value
        pose_y=x.yDistance.value
        pose_z=x.zDistance.value
        speed_x = x.xSpeed.value
        speed_y = x.ySpeed.value
        speed_z = x.zSpeed.value
        acceleration_x = x.xAcceleration.value
        acceleration_y = x.yAcceleration.value
        
        # Create a TrackObject and populate it with information
        TOL=TrackObject()
        TOL.mast_id
        TOL.object_id=ObjectID
        TOL.timestamp
        TOL.box_length_x
        TOL.box_length_y
        TOL.box_length_z
        TOL.pose_x=pose_x
        TOL.pose_y=pose_y
        TOL.pose_z=pose_z
        TOL.latitude
        TOL.longitude
        TOL.altitude
        TOL.orientation_x
        TOL.orientation_y
        TOL.orientation_z
        TOL.vel_x=speed_x
        TOL.vel_y=speed_y
        TOL.vel_z=speed_z
        TOL.acc_x=acceleration_x
        TOL.acc_y=acceleration_y
        TOL.acc_z
        TOL.object_class
        TOL.classification_confidence
        TOL.detection_confidence
        TOL.ego_pose_x
        TOL.ego_pose_y
        TOL.ego_vel_x=Speed
        TOL.ego_orientation_z=WGS84Angle

        # Append the TrackObject to the object list in First Mile object
        FirstmileObject.object_list.append(TOL)

    # Publish the First Mile object
    FirstMilesFusion.publish(FirstmileObject)


if __name__ == '__main__':
    # Initialize the ROS node and set up publishers and subscribers
    rospy.init_node('FirstMile_CPM')
    FirstMilesFusion = rospy.Publisher('/first_mile', TrackObjectList, queue_size=1000)
    rospy.Subscriber('/cpm_received',CPM,First_mile_CPM)

    # Spin and wait for messages
    rospy.spin()
