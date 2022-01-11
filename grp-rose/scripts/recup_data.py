#!/usr/bin/python3

#Import dependecies
import rospy
import cv2
#from rospy import message_filters
import message_filters

#Import msg
from sensor_msgs.msg import Image

rospy.init_node('recup_data', anonymous=True)

def callback(color_image, depth_image):
    print("data")
    print(color_image)
    # cv2.imshow("Color", color_image)
    # cv2.waitKey(3)
    # cv2.imshow("Color", depth_image)
    # cv2.waitKey(3)

#Déclarations subscriber
sub_color = message_filters.Subscriber("/color_topic", Image)
sub_depth = message_filters.Subscriber("/depth_topic", Image)

ts = message_filters.TimeSynchronizer([sub_color, sub_depth], 10)
ts.registerCallback(callback)
# spin() enter the program in a infinite loop
rospy.spin()

# def callbackSubColor(data):
#     print(data)

# def callbackSubDepth(data):
#     print(data)

# #Déclarations subscriber
# sub_color = rospy.Subscriber("/color_topic", Image, callbackSubColor)
# sub_depth = rospy.Subscriber("/depth_topic", Image, callbackSubDepth)

# # spin() enter the program in a infinite loop
# rospy.spin()