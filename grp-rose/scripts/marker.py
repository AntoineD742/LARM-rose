#!/usr/bin/python3

#Import dependecies
import rospy

#Import msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

## publie sur le topic /bottle and /map

## subscribe au topic qui permet de détecter les bouteilles

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=10)

rospy.init_node('marker')

while not rospy.is_shutdown():

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.ns = "test"
    marker.id = 0
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0 # toujours laisser à 1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    marker.lifetime = 200

    publisher.publish(marker)


# spin() enter the program in a infinite loop
rospy.spin()