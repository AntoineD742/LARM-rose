#!/usr/bin/python3

#Import dependecies
import math, rospy, random

#Import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MIN_DISTANCE = 1
ANGLE_DE_VUE = 150
NB_SEQ_TOUR_COMPLET = 126
# Initialize ROS::node
rospy.init_node('move', anonymous=True)

def callback(data):
    min_range = min(data.ranges[round((len(data.ranges)/2)-ANGLE_DE_VUE):round((len(data.ranges)/2)+ANGLE_DE_VUE)])
    if (min_range < MIN_DISTANCE):
        cmd.linear.x = 0.0
        if data.header.seq%(2*NB_SEQ_TOUR_COMPLET) < NB_SEQ_TOUR_COMPLET:
            cmd.angular.z = -0.5
        else:
            cmd.angular.z = 0.5
    else:
        cmd.linear.x = 1.0
        cmd.angular.z = 0.0
     
        
    commandPublisher.publish(cmd)


sub = rospy.Subscriber("base_scan", LaserScan, callback)
commandPublisher = rospy.Publisher( '/cmd_vel', Twist, queue_size=10)
cmd = Twist()

# spin() enter the program in a infinite loop
print("Start moving")
rospy.spin()