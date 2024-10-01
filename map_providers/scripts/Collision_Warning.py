#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from etsi_its_msgs.msg import CAM, PerceivedObject, ListOfPerceivedObjects
from map_manager.msg import MapObject
from ros_collision_detection.msg import CollisionCheckResult
from pdk_ros_msg.msg import pdk_RadarObjectList
from pdk_ros_msg.msg import PerceivedObjectContainer
from pdk_ros_msg.msg import PerceivedObject
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
import math
from math import *
import re
import random

#pub_filtered = rospy.Publisher('filtered_data_topic', pdk_RadarObjectList, queue_size=10)

#from map_providers.scripts import gnss_conversion_service
#from ros_gnss_converter.scripts.gnss_conversion_service import xy2ll


'''
Python version of the inline functions defined in the robot_localization, 
navsat_conversions.h
'''
RADIANS_PER_DEGREE = pi/180.0;
DEGREES_PER_RADIAN = 180.0/pi;

# Grid granularity for rounding UTM coordinates to generate MapXY.
grid_size = 100000.0;    # 100 km grid

# WGS84 Parameters
WGS84_A =6378137.0   # major axis
WGS84_B =6356752.31424518  # minor axis
WGS84_F =0.0033528107    # ellipsoid flattening
WGS84_E =0.0818191908    # first eccentricity
WGS84_EP =0.0820944379    # second eccentricity

# UTM Parameters
UTM_K0  =  0.9996               # scale factor
UTM_FE  = 500000.0             # false easting
UTM_FN_N = 0.0                  # false northing, northern hemisphere
UTM_FN_S = 10000000.0           # false northing, southern hemisphere
UTM_E2   = (WGS84_E*WGS84_E)    # e^2
UTM_E4   = (UTM_E2*UTM_E2)      # e^4
UTM_E6   = (UTM_E4*UTM_E2)      # e^6
UTM_EP2  = (UTM_E2/(1-UTM_E2))  # e'^2


def ll2xy(lat,lon,origin_lat,origin_lon):
    '''
    Geonav: Lat/Long to X/Y
    Convert latitude and longitude in dec. degress to x and y in meters
    relative to the given origin location.  Converts lat/lon and orgin to UTM and then takes the difference

    Args:
      lat (float): Latitude of location
      lon (float): Longitude of location
      orglat (float): Latitude of origin location
      orglon (float): Longitude of origin location

    Returns:
      tuple: (x,y) where...
        x is Easting in m (local grid)
        y is Northing in m  (local grid)
    '''

    outmy, outmx, outmzone = LLtoUTM(origin_lat,origin_lon)
    utmy, utmx, utmzone = LLtoUTM(lat,lon)
    if (not (outmzone==utmzone)):
        print('WARNING: geonav_conversion: origin and location are in different UTM zones!')
    y = utmy-outmy
    x = utmx-outmx
    return (x,y) 

def xy2ll(x, y, orglat, orglon):
    '''
    '''
    outmy, outmx, outmzone = LLtoUTM(orglat,orglon)
    utmy = outmy+y
    utmx = outmx+x
    return UTMtoLL(utmy,utmx,outmzone)

'''*
 * Determine the correct UTM letter designator for the
 * given latitude
 *
 * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 '''
def UTMLetterDesignator(Lat):
    
    LetterDesignator =""

    if ((84 >= Lat) and (Lat >= 72)):  LetterDesignator = 'X'
    
    elif ((72 > Lat) and (Lat >= 64)):  LetterDesignator = 'W';
    elif ((64 > Lat) and (Lat >= 56)):  LetterDesignator = 'V';
    elif ((56 > Lat) and (Lat >= 48)):  LetterDesignator = 'U';
    elif ((48 > Lat) and (Lat >= 40)):  LetterDesignator = 'T';
    elif ((40 > Lat) and (Lat >= 32)):  LetterDesignator = 'S';
    elif ((32 > Lat) and (Lat >= 24)):  LetterDesignator = 'R';
    elif ((24 > Lat) and (Lat >= 16)):  LetterDesignator = 'Q';
    elif ((16 > Lat) and (Lat >= 8)) :  LetterDesignator = 'P';
    elif (( 8 > Lat) and (Lat >= 0)) :  LetterDesignator = 'N';
    elif (( 0 > Lat) and (Lat >= -8)):  LetterDesignator = 'M';
    elif ((-8 > Lat) and (Lat >= -16)): LetterDesignator = 'L';
    elif ((-16 > Lat) and (Lat >= -24)): LetterDesignator = 'K';
    elif ((-24 > Lat) and (Lat >= -32)): LetterDesignator = 'J';
    elif ((-32 > Lat) and (Lat >= -40)): LetterDesignator = 'H';
    elif ((-40 > Lat) and (Lat >= -48)): LetterDesignator = 'G';
    elif ((-48 > Lat) and (Lat >= -56)): LetterDesignator = 'F';
    elif ((-56 > Lat) and (Lat >= -64)): LetterDesignator = 'E';
    elif ((-64 > Lat) and (Lat >= -72)): LetterDesignator = 'D';
    elif ((-72 > Lat) and (Lat >= -80)): LetterDesignator = 'C';
        # 'Z' is an error flag, the Latitude is outside the UTM limits
    else: LetterDesignator = 'Z';
    return LetterDesignator


'''*
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 Retuns a tuple of (UTMNorthing, UTMEasting, UTMZone)
 '''
def LLtoUTM(Lat,Long):

  a = WGS84_A;
  eccSquared = UTM_E2;
  k0 = UTM_K0;

  # Make sure the longitude is between -180.00 .. 179.9
  LongTemp = (Long+180.0)-int((Long+180.)/360.)*360.-180.;

  LatRad = Lat*RADIANS_PER_DEGREE;
  LongRad = LongTemp*RADIANS_PER_DEGREE;
  ZoneNumber = int((LongTemp + 180.0)/6.0) + 1;

  if ( Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
      ZoneNumber = 32;
        # Special zones for Svalbard
  if ( Lat >= 72.0 and Lat < 84.0 ):
      if (      LongTemp >= 0.0  and LongTemp <  9.0 ): ZoneNumber = 31;
      elif ( LongTemp >= 9.0  and LongTemp < 21.0 ): ZoneNumber = 33;
      elif ( LongTemp >= 21.0 and LongTemp < 33.0 ): ZoneNumber = 35;
      elif ( LongTemp >= 33.0 and LongTemp < 42.0 ): ZoneNumber = 37;
  # +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1.0)*6.0 - 180.0 + 3.0;
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

  # Compute the UTM Zone from the latitude and longitude
  UTMZone = "%d%s"%(ZoneNumber,UTMLetterDesignator(Lat))
  #print("UTM Zone: %s"%(UTMZone))
  eccPrimeSquared = (eccSquared)/(1.0-eccSquared);
  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);
  
  M = a*((1 - eccSquared/4.0 - 3.0*eccSquared*eccSquared/64.0
          - 5.0*eccSquared*eccSquared*eccSquared/256.0) * LatRad
         - (3.0*eccSquared/8.0 + 3.0*eccSquared*eccSquared/32.0
            + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(2.0*LatRad)
         + (15.0*eccSquared*eccSquared/256.0
            + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(4.0*LatRad)
         - (35.0*eccSquared*eccSquared*eccSquared/3072.0)*sin(6.0*LatRad));

  UTMEasting = (k0*N*(A+(1.0-T+C)*A*A*A/6.0
                      + (5.0-18.0*T+T*T+72*C
                         - 58.0*eccPrimeSquared)*A*A*A*A*A/120.0)
                + 500000.0)

  UTMNorthing = (k0*(M+N*tan(LatRad)
                     *(A*A/2.0+(5.0-T+9.0*C+4.0*C*C)*A*A*A*A/24.0
                       + (61.0-58.0*T+T*T+600.0*C
                          - 330.0*eccPrimeSquared)*A*A*A*A*A*A/720.0)));
  if (Lat < 0):
      # 10000000 meter offset for southern hemisphere
      UTMNorthing += 10000000.0;
  
  return (UTMNorthing, UTMEasting, UTMZone)

'''*
 * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 Returns (Lat, Lon, UTMZone)
 '''
def UTMtoLL(UTMNorthing,UTMEasting,UTMZone):
  k0 = UTM_K0;
  a = WGS84_A;
  eccSquared = UTM_E2;
  e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));

  x = UTMEasting - 500000.0;  # remove 500,000 meter offset for longitude
  y = UTMNorthing;
  
  ZoneLetter = re.findall('([a-zA-Z])',UTMZone)[0]
  ZoneNumber = float( UTMZone.split(ZoneLetter)[0] )

  if (ZoneLetter <'N'):
      # remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0;

  # +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6.0 - 180.0 + 3.0;
  eccPrimeSquared = (eccSquared)/(1.0-eccSquared);
  M = y / k0;
  mu = M/(a*(1.0-eccSquared/4.0-3.0*eccSquared*eccSquared/64.0
             -5.0*eccSquared*eccSquared*eccSquared/256.0));
  phi1Rad = mu + ((3.0*e1/2.0-27.0*e1*e1*e1/32.0)*sin(2.0*mu)
                  + (21.0*e1*e1/16.0-55.0*e1*e1*e1*e1/32.0)*sin(4.0*mu)
                  + (151.0*e1*e1*e1/96.0)*sin(6.0*mu));
  
  N1 = a/sqrt(1.0-eccSquared*sin(phi1Rad)*sin(phi1Rad));
  T1 = tan(phi1Rad)*tan(phi1Rad);
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
  R1 = a*(1.0-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
  D = x/(N1*k0);
  Lat = phi1Rad - ((N1*tan(phi1Rad)/R1)
                   *(D*D/2.0
                     -(5.0+3.0*T1+10.0*C1-4.0*C1*C1
                       -9.0*eccPrimeSquared)*D*D*D*D/24.0
                     +(61.0+90.0*T1+298.0*C1+45.0*T1*T1-252.0*eccPrimeSquared
                       -3.0*C1*C1)*D*D*D*D*D*D/720.0));
  
  Lat = Lat * DEGREES_PER_RADIAN;
  
  Long = ((D-(1.0+2.0*T1+C1)*D*D*D/6.0
           +(5.0-2.0*C1+28.0*T1-3.0*C1*C1+8.0*eccPrimeSquared+24.0*T1*T1)
           *D*D*D*D*D/120.0)
          / cos(phi1Rad));
  Long = LongOrigin + Long * DEGREES_PER_RADIAN;

  return (Lat, Long)
'''
def filtered_radar_object_list_callback(data):
    # Create a new marker
    marker = Marker()
    marker.header = data.header
    marker.header.frame_id = "base_footprint"
    marker.lifetime = rospy.Duration(0.7)
    if data.f_ObjectScore > 0.90 and -50 < data.f_DistX < 50 and -50 < data.f_DistY < 50:
        marker.ns = "radar_objects"
        marker.id = data.u_ObjId  # Use a unique ID for the marker
        marker.action = Marker.ADD
        # Set the marker position and orientation to represent the whole object list
    
        marker.type = Marker.ARROW
        marker.color.r = 1.0 # Use the first object's color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = data.f_DistX  # Use the first object's position
        marker.pose.position.y = data.f_DistY
        marker.pose.position.z = 1
        yaw = math.atan2(data.f_VabsY, data.f_VabsX)
        q = quaternion_from_euler(0, 0, yaw)
        
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Calculate the overall size of the marker based on the object list
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
'''
def location(data):
    global location_pos
    location_pos= NavSatFix()
    

    location_pos.header=data.header
    location_pos.altitude=data.altitude
    location_pos.latitude=data.latitude
    location_pos.longitude=data.longitude

def collision (data):
    map_object=MapObject()
    header=data.header
    PrcvdObjects=data.perceived_object
    ObjectType=PrcvdObjects.object_type
    ObjectMovement=PrcvdObjects.object_movement
    Header=ObjectMovement.header
    Id=ObjectMovement.id
    Position_X=ObjectMovement.position.x
    Position_Y=ObjectMovement.position.y
    
    Heading=ObjectMovement.heading
    Speed=ObjectMovement.speed
    Acceleration=ObjectMovement.acceleration
    TTC=data.ttc
    ResultType=data.result_type
    

    #print(TTC)
    print(ResultType)

    
        
    converted_obj_pose = xy2ll(x=Position_X, y=Position_Y, 
                                orglat=location_pos.latitude, orglon=location_pos.longitude)
    #print(converted_obj_pose)
    #print(type(converted_obj_pose[0]))
    #map_object.source_id = obj.tracking_id
    map_object.source_id = int(TTC)
    #print(map_object.source_id)
    map_object.position.latitude = int(converted_obj_pose[0] * (10**7))
    #print(map_object.position.latitude) 
    map_object.position.longitude = int(converted_obj_pose[1] * (10**7))
    #print(map_object.position.longitude)
    if ResultType==0:
        map_object.type="Level0"
    elif ResultType==1:
        map_object.type="Level1"
    elif ResultType==2:
        map_object.type="Level2"
    else:
        map_object.type="Level3"        
    #map_object.type = "Collision"
    print(map_object.type)
    map_object.source="Collision_Warning"
    #print(map_object.source)
    map_object.expiration_time = 1.3
    #print(map_object.expiration_time)

    map_publisher.publish(map_object)
    
'''
def map_pdk_objects(data):

    map_object = MapObject()
    number_of_objects=data.numberOfPerceivedObjects
    #print(number_of_objects)
    prcvd_objects=data.perceivedObjects
    #print(prcvd_objects)
    for x in prcvd_objects:
        ID=x.objectID
        #print(ID)
        Delta_time=x.measurementDeltaTime
        #print(Delta_time)
        pose_x = x.position.x_cord.value/100
        print('x',pose_x)
        pose_y = x.position.y_cord.value/100
        print(pose_y)
        converted_obj_pose = xy2ll(x=pose_x, y=pose_y, 
                                orglat=location_pos.latitude, orglon=location_pos.longitude)
        #print(converted_obj_pose)
        #print(type(converted_obj_pose[0]))
        #map_object.source_id = obj.tracking_id
        map_object.source_id = random.randint(0,20)
        #print(map_object.source_id)
        map_object.position.latitude = int(converted_obj_pose[0] * (10**7))
        #print(map_object.position.latitude) 
        map_object.position.longitude = int(converted_obj_pose[1] * (10**7))
        #print(map_object.position.longitude)
        map_object.type = "PDK"
        print(map_object.type)
    map_object.source="pdk_tracking"
    print(map_object.source)
    map_object.expiration_time = 1.3
    print(map_object.expiration_time)

    map_publisher.publish(map_object)
'''
if __name__ == '__main__':
    rospy.init_node('Collision_vis_tracker')
    map_publisher = rospy.Publisher('/map_manager', MapObject, queue_size=1000)
    rospy.Subscriber('/gnss/fix',NavSatFix,location)
    #rospy.Subscriber('/car2x_unit',PerceivedObjectContainer,map_pdk_objects)
    rospy.Subscriber('/collision_warning',CollisionCheckResult,collision)
    # Spin and wait for messages
    rospy.spin()
