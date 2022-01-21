#!/usr/bin/python3

#####################################################################################################################################################################
#                                                                           ALGORITHME DE DETECTION DE BOUTEILLES
#                                                                               PAR SEGMENTATION DES COULEURS 
#####################################################################################################################################################################


#####################################################################################################################################################################
#                                                                                       IMPORTS
#####################################################################################################################################################################
from __future__ import print_function
import sys
import rospy, tf
import cv2
import numpy as np
import image_geometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#####################################################################################################################################################################
#                                                                                      PARAMETRES
#####################################################################################################################################################################

#Seuils de Hue-Saturation-Value             H: 0-179, S: 0-255, V: 0-255
lowBadge=np.array([0, 35, 100])             #Badge bouteille noire (Min)
hiBadge=np.array([6, 255, 255])             #Badge bouteille noire (Max)

lowRuban = np.array([0, 0, 230])            #Ruban bouteille noire (Min)
hiRuban = np.array([255, 100, 255])         #Ruban bouteille noire (Max)

lowOrangeBottle = np.array([0, 230, 230])   #Bouteille orange (Min)
hiOrangeBottle = np.array([25, 255, 255])   #Bouteille orange (Min)

#Seuil de profondeur (0-255)
Threshold_Param = 100           

#Taille de la bouteille en pixel
NBR_PIXEL_MIN_DETECTION_BOUTEILLE_ORANGE = 5000       
NBR_PIXEL_MAX_DETECTION_BOUTEILLE_ORANGE = 20000

class bottleFinder: 
    #####################################################################################################################################################################
    #                                                                  RECUPERATION DONNEES CAMERA (Depth, RGB)
    #                                                                           FILTRAGE ADAPTE
    #                                                                        DETECTION BOUTEILLES
    #                                                                        CALCUL DES COORDONNES
    #                                                                        ENVOI A /coord_bottle
    #####################################################################################################################################################################
    def __init__(self):
        self.bridge = CvBridge()            #Conversion Images OpenCV-ROS
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callbackDepth)  #Recuperation image de profondeur (Alignée)
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callbackColor)                   #Recuperation image RGB
        self.bottle_pub = rospy.Publisher("/coord_bottle", PoseStamped, queue_size = 10)                            #Envoi a /coor_bottle

        self.color_map = None   #Framde Depth
        self.depth_map = None   #Frame Color

        # Initialisation des variables pour calculer les coordonnées de la bouteille dans le référentiel de la camera
        self.camera = image_geometry.PinholeCameraModel()   
        self.camera_info = CameraInfo()
        self.camera_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.callbackCamera)

        self.coord_bottles = PoseStamped()  #Données à envoyer à /coord_bottle

    def callbackCamera(self, data):
        # Récupération des données de la caméra
        self.camera_info = data

    def callbackDepth(self,data):
        try:
            #Recuperation image de profondeur (Alignée), Conveersion au format OpenCV
            self.depth_map = self.bridge.imgmsg_to_cv2(data, "passthrough")
            #Recherche des bouteilles
            self.find_bottles()
        except CvBridgeError as e:
            print(e)

    def callbackColor(self,data):
        try:
            #Recuperation image RGB, Conversion au format OpenCV
            self.color_map = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError as e:
            print(e)
    
    def find_bottles(self):
        if self.depth_map is not None and self.color_map is not None:
            #Conversion depth
            depth_copy = cv2.convertScaleAbs(self.depth_map, alpha=0.1)

            #Seuil depth
            discarded, maskProfondeur = cv2.threshold(depth_copy,Threshold_Param,255,cv2.THRESH_BINARY)
            #Transformation du seuil en mask
            maskProfondeur = cv2.merge([maskProfondeur,maskProfondeur,maskProfondeur])
            maskProfondeur=cv2.inRange(maskProfondeur, 0 , 254)
            
            #Application du seuil de profondeur à l'image RGB
            thresholded_color = cv2.bitwise_and(self.color_map, self.color_map, mask= maskProfondeur)

            #Conversion couleur en HSV
            img_hsv = cv2.cvtColor(thresholded_color, cv2.COLOR_BGR2HSV)

            #Mask Ruban (Double seuillage puis amelioration)
            maskRuban=cv2.inRange(img_hsv, lowRuban, hiRuban)   
            maskRuban=cv2.erode(maskRuban, None, iterations=1)
            maskRuban=cv2.dilate(maskRuban, None, iterations=1)

            #Mask bouteille orange
            maskBouteilleOrange=cv2.inRange(img_hsv, lowOrangeBottle, hiOrangeBottle)
            maskBouteilleOrange=cv2.erode(maskBouteilleOrange, None, iterations=3)
            maskBouteilleOrange=cv2.dilate(maskBouteilleOrange, None, iterations=3)
            mask = maskBouteilleOrange

            #Extraction des zones d'interets
            img_result=cv2.bitwise_and(thresholded_color, thresholded_color, mask= mask)

            #Detection pixels oranges 
            grayCounter = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
            nbrPixelsDetectes = cv2.countNonZero(grayCounter)

            if nbrPixelsDetectes > NBR_PIXEL_MIN_DETECTION_BOUTEILLE_ORANGE and nbrPixelsDetectes < NBR_PIXEL_MAX_DETECTION_BOUTEILLE_ORANGE: #Critere de taille
                #1ere detection des contours
                contours, hierarchy = cv2.findContours(grayCounter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) <  10 and len(contours) > 0:       #Si + que 10 contours, il s'agit d'une fausse detection du sol
                    #On trouve un nombre faible de contours, on les fusionne avec un blur 
                    grayCounter=cv2.erode(grayCounter, None, iterations=9)
                    grayCounter=cv2.dilate(grayCounter, None, iterations=9)
                    #2eme detecton des contours
                    contours, hierarchy = cv2.findContours(grayCounter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    
                    #Calcul bbox de la bouteille puis de son centre
                    bbox_bottle = cv2.boundingRect(contours[0])
                    x_cnt_bottle = int(bbox_bottle[0] + bbox_bottle[2] // 2)
                    y_cnt_bottle = int(bbox_bottle[1] + bbox_bottle[3] // 2)
                    center_bottle = (x_cnt_bottle, y_cnt_bottle)

                    #Conversion des coordonnées de la bouteille sur l'image en coordonnées par rapport à "camera_color_optical_frame"
                    self.camera.fromCameraInfo(self.camera_info)
                    distance_from_camera = self.depth_map[y_cnt_bottle, x_cnt_bottle]/1000
                    coord_map = (self.camera.projectPixelTo3dRay(center_bottle))

                    self.coord_bottles.header.frame_id = "camera_color_optical_frame"

                    # On met à jour la variable self.coord_bottles
                    self.coord_bottles.pose.position.x = coord_map[0]*distance_from_camera
                    self.coord_bottles.pose.position.y = coord_map[1]*distance_from_camera
                    self.coord_bottles.pose.position.z = coord_map[2]*distance_from_camera

                    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                    self.coord_bottles.pose.orientation.x = quaternion[0]
                    self.coord_bottles.pose.orientation.y = quaternion[1]
                    self.coord_bottles.pose.orientation.z = quaternion[2]
                    self.coord_bottles.pose.orientation.w = quaternion[3]
                    
                    #Envoi des coordonnées sur le topic /coord_bottle
                    self.bottle_pub.publish(self.coord_bottles)


            #Affichage
            cv2.imshow("Result", img_result)
            cv2.waitKey(3)
            

def main(args):
  bottle_finder = bottleFinder()
  rospy.init_node('bottleFinder', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)