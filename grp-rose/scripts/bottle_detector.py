#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#Constantes seuils
lowBadge=np.array([0, 35, 100]) 
hiBadge=np.array([6, 255, 255])

lowRuban = np.array([0, 0, 230]) 
hiRuban = np.array([255, 100, 255])

Threshold_Param = 50


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callbackDepth)
        self.color_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callbackColor)

        self.color_map = None
        self.depth_map = None

    def callbackDepth(self,data):
        try:
            self.depth_map = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.find_bottles()
        except CvBridgeError as e:
            print(e)

    def callbackColor(self,data):
        try:
            self.color_map = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            print(e)
    
    def find_bottles(self):
        if self.depth_map is not None and self.color_map is not None:
            #Conversion depth
            self.depth_map = cv2.convertScaleAbs(self.depth_map, alpha=0.1)

            #Threshold depth
            discarded, self.depth_map = cv2.threshold(self.depth_map,Threshold_Param,255,cv2.THRESH_BINARY)

            #Conversion couleur
            img_hsv = cv2.cvtColor(self.color_map, cv2.COLOR_BGR2HSV)

            #Mask Badge (Double seuillage puis amelioration)
            maskBadge=cv2.inRange(img_hsv, lowBadge, hiBadge)
            maskBadge=cv2.erode(maskBadge, None, iterations=1)
            maskBadge=cv2.dilate(maskBadge, None, iterations=1)

            #Mask Ruban (Double seuillage puis amelioration)
            maskRuban=cv2.inRange(img_hsv, lowRuban, hiRuban)
            maskRuban=cv2.erode(maskRuban, None, iterations=1)
            maskRuban=cv2.dilate(maskRuban, None, iterations=1)

            #Combinaison des masks
            mask = cv2.add(maskBadge, maskRuban)

            #Extraction des zones d'interets
            img_result=cv2.bitwise_and(self.color_map, self.color_map, mask= mask)
            
            #Affichage
            cv2.imshow("Depth", self.depth_map)
            cv2.waitKey(3)
            cv2.imshow("Color", img_result)
            cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)