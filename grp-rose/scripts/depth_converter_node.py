#!/usr/bin/env python
#IMPORTS
from ros2opencv import depth_image_converter
from ros2opencv import color_image_converter
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main(args):
  dic = depth_image_converter()
  rospy.init_node('depth_image_converter')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)