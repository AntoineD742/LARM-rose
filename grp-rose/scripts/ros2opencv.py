#!/usr/bin/env python
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class depth_image_converter:

  def __init__(self):
    self.depth_pub = rospy.Publisher("depth_topic",Image, queue_size=10)

    self.bridge = CvBridge()
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callbackDepth)
    
  def callbackDepth(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

      #Conversion Alex Antoine
      cv_image = cv2.convertScaleAbs(cv_image, alpha=0.1)
      cv2.imshow("Depth", cv_image)
      cv2.waitKey(3)
      # Accès pixel central
      # print(cv_image[240,424])
      self.depth_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))
    except CvBridgeError as e:
      print(e)


class color_image_converter:

  def __init__(self):
    self.color_pub = rospy.Publisher("color_topic",Image, queue_size=10)

    self.bridge = CvBridge()
    self.color_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callbackColor)

  def callbackColor(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      cv2.imshow("Color", cv_image)
      cv2.waitKey(3)
      # Accès pixel central
      # print(cv_image[240,424])
      self.color_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)















# def main(args):
#   ic = image_converter()
#   rospy.init_node('image_converter', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)