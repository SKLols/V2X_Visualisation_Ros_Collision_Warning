import rospy
from std_msgs.msg import String
from etsi_its_msgs.msg import CAM, PerceivedObject, ListOfPerceivedObjects, 
from map_manager.msg import MapObject
from uuid import uuid4
from traffic_monitoring.msg import GlobalTrackObject, GlobalTrackObjectList
#from gnss_conversion_service import xy2ll
from math import *
import re
from etsi_its_msgs.msg import _StationType

class RadarReceiver:
    def __init__(self, publisher):
        
        self.m_publisher = publisher
        rospy.init_node('radar_reciever_subs')
        rospy.loginfo("Subscriber for radar_rosbag started")
        self.m_sub = rospy.Subscriber('/pdk/tracking/filtered_marker_array', GlobalTrackObjectList, self.receive_fusion_data_fields)


    def receive_fusion_data_fields(self, data):


        map_object = MapObject()

        header = data.header
        frame_id = data.frame_id
        rsu_sensor_id = data.rsu_sensor_id
        total_objects = data.total_objects
        info = data.info
        object_list = data.object_list
        map_object.header = header
        #print(header.type)
        #print(map_object.header.type)

        for obj in object_list:
            pose_x = obj.obj_box_x
            pose_y = obj.obj_box_y
            lat = obj.obj_lat
            lon =obj.obj_lon
            obj_class = obj.obj_class
            #print(obj_class)
            converted_obj_pose = xy2ll(x=obj.obj_box_x, y=obj.obj_box_y, 
                                       orglat=obj.obj_lat, orglon=obj.obj_lon)
            #print(converted_obj_pose)
            #print(type(converted_obj_pose[0]))
            map_object.source_id = obj.tracking_id
            
            map_object.position.latitude = int(converted_obj_pose[0] * (10**7)) 
            map_object.position.longitude = int(converted_obj_pose[1] * (10**7))
            #map_object.position_latitude = converted_obj_pose[0]
            #map_object.position_longitude = converted_obj_pose[1]
            #map_object.type = "CAR"
            map_object.type = obj.obj_class
            map_object.id = str(obj.mast_id)

            #if _StationType == PerceivedObjec

        
        
        #print(map_object.type)
        #map_object.position = cam.reference_position  # In micro degree * 10 (from standard)
        #map_object.heading = cam.high_frequency_container.heading

        

        """
        if cam.station_type.value == CAM.PASSENGER_CAR:
            map_object.type = "VEHICLE"
        elif cam.station_type.value == CAM.LIGHT_TRUCK or cam.station_type.value == CAM.HEAVY_TRUCK:
            map_object.type = "TRUCK"
        elif cam.station_type.value == CAM.MOTORCYCLE:
            map_object.type = "MOTORCYCLE"
        elif cam.station_type.value == CAM.PEDESTRIAN:
            map_object.type = "PEDESTRIAN"
        elif cam.station_type.value == CAM.CYCLIST:
            map_object.type = "CYCLIST"
        elif cam.station_type.value == CAM.ROAD_SIDE_UNIT:
            map_object.type = "RSU"
        else:
            map_object.type = "UNKNOWN

        if cam.its_header.station_id in self.m_cams_received:
            map_object.id = self.m_cams_received[cam.its_header.station_id]
        else:
            map_object.id = str(uuid4())
            self.m_cams_received[cam.its_header.station_id] = map_object.id
        """

        map_object.source = "fusion_data"
        

        map_object.expiration_time = 1.3  # in seconds
        #print(map_object)

        self.m_publisher.publish(map_object)
        #print('shat', self.m_publisher)


    


if __name__ == '__main__':

    #rospy.init_node('cam_provider')
    publisher = rospy.Publisher("map_manager", MapObject, queue_size=1000)

    fusion_receiver = FusionReceiver(publisher)

    rospy.spin()
