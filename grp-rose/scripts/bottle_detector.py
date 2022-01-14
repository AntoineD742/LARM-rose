#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
import image_geometry
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#Constantes seuils
lowBadge=np.array([0, 35, 100]) 
hiBadge=np.array([6, 255, 255])

lowRuban = np.array([0, 0, 230]) 
hiRuban = np.array([255, 100, 255])

# lowOrangeBottle = np.array([0, 180, 200]) 
lowOrangeBottle = np.array([0, 200, 200]) 
# hiOrangeBottle = np.array([25, 255, 255])
hiOrangeBottle = np.array([25, 255, 255])
#HSV: 85 110 ; 240 255, 230 , 255

Threshold_Param = 100

NBR_PIXEL_MIN_DETECTION_BOUTEILLE_ORANGE = 5000
NBR_PIXEL_MAX_DETECTION_BOUTEILLE_ORANGE = 20000

# class bottle:
#     def __init__(self):
#         self.x_img = None
#         self.y_img = None
#         self.x_relative = None
#         self.y_relative = None
#         self.x_map = None
#         self.y_map = None


class image_converter:                                          # CHANGER LE NOM

    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callbackDepth)
        self.color_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callbackColor)
        self.bottle_pub = rospy.Publisher("/bottle", Vector3, queue_size = 10)

        self.color_map = None
        self.depth_map = None


        self.camera = image_geometry.PinholeCameraModel()
        self.camera_info = CameraInfo()
        self.camera_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.callbackCamera)

        self.coord_bottles = Vector3()

    def callbackCamera(self, data):
        self.camera_info = data

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
            depth_copy = cv2.convertScaleAbs(self.depth_map, alpha=0.1)

            #Threshold depth
            discarded, maskProfondeur = cv2.threshold(depth_copy,Threshold_Param,255,cv2.THRESH_BINARY)
            maskProfondeur = cv2.merge([maskProfondeur,maskProfondeur,maskProfondeur])
            maskProfondeur=cv2.inRange(maskProfondeur, 0 , 254)
            
            #Application du seuil de profondeur à l'image RGB
            thresholded_color = cv2.bitwise_and(self.color_map, self.color_map, mask= maskProfondeur)

            #Conversion couleur en HSV
            img_hsv = cv2.cvtColor(thresholded_color, cv2.COLOR_BGR2HSV)

            #Mask Badge (Double seuillage puis amelioration)
            # maskBadge=cv2.inRange(img_hsv, lowBadge, hiBadge)
            # maskBadge=cv2.erode(maskBadge, None, iterations=1)
            # maskBadge=cv2.dilate(maskBadge, None, iterations=1)

            #Mask Ruban (Double seuillage puis amelioration)
            maskRuban=cv2.inRange(img_hsv, lowRuban, hiRuban)   
            maskRuban=cv2.erode(maskRuban, None, iterations=1)
            maskRuban=cv2.dilate(maskRuban, None, iterations=1)


            #Mask bouteille orange
            maskBouteilleOrange=cv2.inRange(img_hsv, lowOrangeBottle, hiOrangeBottle)
            maskBouteilleOrange=cv2.erode(maskBouteilleOrange, None, iterations=1)
            maskBouteilleOrange=cv2.dilate(maskBouteilleOrange, None, iterations=1)
            #Combinaison des masks
            # mask = cv2.add(maskRuban, maskBouteilleOrange)
            # mask = cv2.add(maskBadge, maskRuban)
            mask = maskBouteilleOrange

            #Extraction des zones d'interets
            img_result=cv2.bitwise_and(thresholded_color, thresholded_color, mask= mask)
            #Detection pixels oranges (Tri par taille)
            grayCounter = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)

            nbrPixelsDetectes = cv2.countNonZero(grayCounter)
            if nbrPixelsDetectes > NBR_PIXEL_MIN_DETECTION_BOUTEILLE_ORANGE and nbrPixelsDetectes < NBR_PIXEL_MAX_DETECTION_BOUTEILLE_ORANGE: #Critere max de taille?
                contours, hierarchy = cv2.findContours(grayCounter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #grayCounter = cv2.drawContours(self.color_map, contours, -1, (0,255,0), -1)
                # rospy.loginfo("Nbr contours: %s", len(contours))
                if len(contours) <  10 and len(contours) > 0:
                    #On trouve un nombre faible de contours, on fusionne les contours avec un blur 
                    grayCounter=cv2.erode(grayCounter, None, iterations=9)
                    grayCounter=cv2.dilate(grayCounter, None, iterations=9)
                    contours, hierarchy = cv2.findContours(grayCounter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    
                    bbox_bottle = cv2.boundingRect(contours[0])
                    x_cnt_bottle = int(bbox_bottle[0] + bbox_bottle[2] // 2)
                    y_cnt_bottle = int(bbox_bottle[1] + bbox_bottle[3] // 2)
                    center_bottle = (x_cnt_bottle, y_cnt_bottle)

                    #SUPPRIMER LES 4 LIGNES EN DESSOUS
                    radius = 10
                    color = (255, 0, 0)
                    thickness = 2
                    image = cv2.circle(img_result, center_bottle, radius, color, thickness)

                    self.camera.fromCameraInfo(self.camera_info)
                    distance_from_camera = self.depth_map[y_cnt_bottle, x_cnt_bottle]/1000
                    coord_map = self.camera.projectPixelTo3dRay(center_bottle)
                    # MULTPILCATION PAR LA DISTANCE ???????????????????????
                    self.coord_bottles.x = coord_map[0]*distance_from_camera
                    self.coord_bottles.y = coord_map[1]*distance_from_camera
                    self.coord_bottles.z = 0.05
                    
                    self.bottle_pub.publish(self.coord_bottles)


                    #self.bottle_pub.publish(str(depth_map[contours[0][0][0], contours[0][0][1]]))
                    # AVOIR LES COORDONNEES D'UNE BOUTEILLE
                    # MELANGER LES COUNTOURS / EN CHOISIR UN SEUL
                    # PUBLISH SUR LE TOPIC
                    # VERIFIER QU'UNE BOUTEILLE NE SE TROUVE PAS DEJA ICI
                

            #Affichage
            # cv2.imshow("Depth", self.depth_map)
            # cv2.waitKey(3)
            cv2.imshow("Color", self.color_map)
            cv2.waitKey(3)
            # cv2.imshow("maskProfondeur", maskProfondeur)
            # cv2.waitKey(3)
            cv2.imshow("grayCounter", grayCounter)
            cv2.waitKey(3)
            cv2.imshow("Result", img_result)
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