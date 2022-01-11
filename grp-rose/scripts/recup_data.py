#!/usr/bin/python3

#Import dependecies
import rospy

#Import msg
from sensor_msgs.msg import Image

rospy.init_node('recup_data', anonymous=True)

def callbackSubColor(data):
    print(data)

def callbackSubDepth(data):
    print(data)

#Déclarations subscriber
sub_color = rospy.Subscriber("/color_topic", Image, callbackSubColor)
sub_depth = rospy.Subscriber("/depth_topic", Image, callbackSubDepth)

# spin() enter the program in a infinite loop
rospy.spin()