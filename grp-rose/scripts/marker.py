#!/usr/bin/python3

#Import dependecies
import rospy

#Import msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

## publie sur le topic /bottle and /map

## subscribe au topic qui permet de détecter les bouteilles

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, Marker, queue_size=10)

rospy.init_node('marker')

def callback():
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0 # toujours laisser à 1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 1.0
    marker.pose.position.y = 1.0
    marker.pose.position.z = 1.0

    publisher.publish(marker)

sub = rospy.Subscriber("/move_base_simple/goal", Vector3, callback)

# spin() enter the program in a infinite loop
rospy.spin()