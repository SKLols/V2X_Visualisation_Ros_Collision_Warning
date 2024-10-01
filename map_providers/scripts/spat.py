import rospy
import std_msgs
import etsi_its_msgs.msg as its_msg
import socket
import sys
import json
import asn1tools
import regex as re
from math import *


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 7200
    vanetza_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    vanetza_sock.bind((UDP_IP, UDP_PORT))
    foo = asn1tools.compile_files('map_spat.asn', 'uper')
    
    while not rospy.is_shutdown():
        data = vanetza_sock.recvfrom(2000)
        spat_msg = foo.decode("SPATEM", data[0])

        print(spat_msg)
   

if __name__ == '__main__':
    listener()