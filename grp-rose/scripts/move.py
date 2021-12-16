#!/usr/bin/python3

#Import dependecies
import math, rospy

#Import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MIN_DISTANCE = 1

# Initialize ROS::node
rospy.init_node('move', anonymous=True)


def callback(data):
    #rospy.loginfo(len(data.ranges))
    min_range = min(data.ranges[round((len(data.ranges)/2)-100):round((len(data.ranges)/2)+100)])
    #rospy.loginfo(min_range)
    if (min_range < MIN_DISTANCE):
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
    else:
        cmd.linear.x = 1.0
        cmd.angular.z = 0.0
    commandPublisher.publish(cmd)


sub =rospy.Subscriber("base_scan", LaserScan, callback)
commandPublisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=10)
cmd = Twist()

# spin() enter the program in a infinite loop
print("Start moving")
rospy.spin()