#!/usr/bin/python3

#Import dependecies
import rospy

#Import msg
from geometry_msgs.msg import Vector3

publisher = rospy.Publisher("/bottle", Vector3, queue_size=10)

coord = Vector3(0.0, 0.0, 0.0)


rospy.init_node('bottle')

while not rospy.is_shutdown():
    publisher.publish(coord)

# spin() enter the program in a infinite loop
rospy.spin()