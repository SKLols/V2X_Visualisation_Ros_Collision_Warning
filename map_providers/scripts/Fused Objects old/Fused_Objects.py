#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from etsi_its_msgs.msg import CAM, PerceivedObject, ListOfPerceivedObjects, CPM
from map_manager.msg import MapObject
from ros_collision_detection.msg import CollisionCheckResult, PerceivedObjects, PerceivedObjectMotion, ObjectMovement
#from pdk_ros_msg.msg import pdk_RadarObjectList
#from pdk_ros_msg.msg import PerceivedObjectContainer
#from pdk_ros_msg.msg import PerceivedObject
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

def location(data):
    global location_pos
    location_pos= NavSatFix()
    

    location_pos.header=data.header
    location_pos.altitude=data.altitude
    location_pos.latitude=data.latitude
    location_pos.longitude=data.longitude


def fused_objects_CPM (data):
    FusedObject=PerceivedObjects()
    Header=data.header
    ItsPduHeader=data.its_header
    DeltaTime=data.generation_delta_time
    StationType=data.station_type
    ReferencePosition=data.reference_position
    OriginatingVehicleContainer=data.originatingVehicleContainer
    SensorInformationContainer=data.sensorInformationContainer
    ListOfPerceivedObjects=data.listOfPerceivedObjects
    number_of_obj=data.numberOfPerceivedObjects
    PerceivedObject=ListOfPerceivedObjects.perceivedObjectContainer
    #number_of_objects=PerceivedObjectContainer.numberOfPerceivedObjects
    #print(PerceivedObjects)
    for x in PerceivedObject:
        ObjectID=x.objectID
        #for y in PerceivedObject:
        global pose_x, pose_y, speed_x, speed_y, acceleration_x, acceleration_y
        #ObjectID=y.objectID
        #print(ObjectID)
        pose_x = x.xDistance.value
        #print('x',pose_x)
        pose_y = x.yDistance.value
        #print('y',pose_y)
        #converted_obj_pose = xy2ll(x=pose_x, y=pose_y, 
        #                        orglat=location_pos.latitude, orglon=location_pos.longitude)
        #print(converted_obj_pose)
        speed_x = x.xSpeed.value
        speed_y = x.ySpeed.value
        acceleration_x = x.xAcceleration.value
        acceleration_y = x.yAcceleration.value
        #print(type(converted_obj_pose[0]))
        #for obj in ObjectMovement:
        FusedObject.header=data.header
        POM= PerceivedObjectMotion()
        POM.object_type = 'CYCLIST'
        POM.object_movement.position.x=pose_x
        #print('pom_x',POM.object_movement.position.x)
        POM.object_movement.position.y=pose_y
        #print('pom_y',POM.object_movement.position.y)
        POM.object_movement.speed=speed_x
        POM.object_movement.acceleration=acceleration_x
        POM.x_length=5.0
        POM.y_length=2.0

        FusedObject.perceived_objects= [POM]
        print(FusedObject.perceived_objects)


    
    


    #map_publisher.publish(FusedObject)
        CPMFusion.publish(FusedObject)
    


if __name__ == '__main__':
    rospy.init_node('FusedObjects_CPM')
    CPMFusion = rospy.Publisher('/fused_objects', PerceivedObjects, queue_size=1000)
    #map_publisher = rospy.Publisher('/fused_objects_1', PerceivedObjectMotion, queue_size=1000)
    rospy.Subscriber('/gnss/fix',NavSatFix,location)
    rospy.Subscriber('/cpm_received',CPM,fused_objects_CPM)
    # Spin and wait for messages
    rospy.spin()
