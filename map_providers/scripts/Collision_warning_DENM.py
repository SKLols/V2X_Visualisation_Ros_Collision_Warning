#!/usr/bin/env python

import rospy
from etsi_its_msgs.msg import PerceivedObject, ListOfPerceivedObjects, CPM, DENM
from ros_collision_detection.msg import CollisionCheckResult
import math
from math import *
import random

# Define a function to process CPM messages and convert them to First Mile format
def CollisionWarning_DENM (data):
    CWDENM=DENM()
    
    # Extract relevant information from the Collison Warning message
    header=data.header
    PrcvdObjects=data.perceived_object
    ObjectType=PrcvdObjects.object_type
    ObjectMovement=PrcvdObjects.object_movement
    Header=ObjectMovement.header
    Id=ObjectMovement.id
    Position_X=ObjectMovement.position.x
    Position_Y=ObjectMovement.position.y
    Xlength=PrcvdObjects.x_length
    Ylength=PrcvdObjects.y_length
    
    Heading=ObjectMovement.heading
    Speed=ObjectMovement.speed
    Acceleration=ObjectMovement.acceleration
    TTC=data.ttc
    ResultType=data.result_type
    
    
    # Populate the DENM msg with relevant information
    CWDENM.header=data.header
    print(CWDENM.header)
    CWDENM.its_header.protocol_version
    CWDENM.its_header.message_id
    CWDENM.its_header.station_id
    CWDENM.management.action_id.station_id
    CWDENM.management.action_id.sequence_number
    CWDENM.management.detection_time
    CWDENM.management.reference_time
    CWDENM.management.termination
    CWDENM.management.event_position.latitude #reference position
    CWDENM.management.event_position.longitude #reference position
    CWDENM.management.event_position.position_confidence #reference position
    CWDENM.management.event_position.altitude #reference position
    CWDENM.management.relevance_distance.value
    CWDENM.management.relevance_traffic_direction.value
    CWDENM.management.validity_duration
    CWDENM.management.transmission_interval
    CWDENM.management.station_type.value
    CWDENM.situation
    CWDENM.location

    
    # Publish the First Mile object
    CollisionWarningDENM.publish(CWDENM)


if __name__ == '__main__':
    # Initialize the ROS node and set up publishers and subscribers
    rospy.init_node('CW_DENM')
    CollisionWarningDENM = rospy.Publisher('Collision_DENM',DENM, queue_size=1000)
    rospy.Subscriber('/collision_warning',CollisionCheckResult,CollisionWarning_DENM)

    # Spin and wait for messages
    rospy.spin()
