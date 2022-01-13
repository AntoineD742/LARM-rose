#!/usr/bin/python3

#Import dependecies
import rospy

#Import msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

## subscribe au topic /bottle

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker, queue_size=10)

rospy.init_node('marker')

def callback(data):

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0 # toujours laisser à 1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    # marker.pose.position.x = 0.0
    # marker.pose.position.y = 0.0
    # marker.pose.position.z = 0.05

    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = data.z

    marker.lifetime.secs = 200

    publisher.publish(marker)

sub = rospy.Subscriber("/bottle", Vector3, callback)

# spin() enter the program in a infinite loop
rospy.spin()