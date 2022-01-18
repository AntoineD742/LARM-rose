#! /usr/bin/python3 
#===================================================
# portage de l'utilisation du DNN OpenCV avec Python
#...................................................
# J.BOONAERT / UV LARM / Jan. 2022
#===================================================
import cv2                  # -->traitement d'image ET machine learning (dont DNN utilise ici)
import numpy as np          # -->a associer a OpenCV pour la manipulation des images
import math                 # -->toujour utile...
import rospkg




#ALEX
import sys
import rospy, tf
import image_geometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError




# get an instance of RosPack with the default search paths
def get_pkg_path():
    rospack = rospkg.RosPack()
    return rospack.get_path('larm_dnn')

#........................................... 
# poids et parametres du modele a utiliser :
# .......................................... 
szUsedModel = get_pkg_path() + "/dnn-model/MobileNetSSD_deploy" # ->nom "de base" du modele pre-entraine
szModelName = szUsedModel + ".caffemodel"        #   dont les constituant sont separes au
szModelConfig = szUsedModel + ".prototxt"        #   sein de ces deux fichiers.
dbMeanVal = 127.5                                       # ->utilise pour la fonction de construction des
dbScaleFactor = 0.007843                                #   "blob" que necessite l'utilisation de ce DNN
# indice pour decoder la sortie du reseau               #    avec OpenCV
iCLASS_ID = 1                                           # ->indice correspondant a l'identifiant (entier) de la classe
iCLASS_CONFIDENCE = 2                                   # ->indice correspondant a la probabilité d'appartenance a la classe
                                                        #   (selon le "point de vue", parfois discutable, du reseau lui-meme...)
iX_TOPLEFT = 3                                          # ->indice de l'abscisse du coin superieur gauche de la fenetre de detection d'un objet
iY_TOPLEFT = 4                                          # ->indice de l'ordonnee....
iX_LOWRIGHT = 5                                         # ->indice de l'abscisse du coin inferieur droit de la fenetre de detecton d'un objet
iY_LOWRIGHT = 6                                         # ->indice de l'ordonnee
# taille des images presentees au reseau
iNET_INPUT_WIDTH = 300                                  # ->pour se conformer a la taille des donnees d'entre du reseau DNN
iNET_INPUT_HEIGHT = 300                                 #   (idem)
# autres parametres
iSLEEP_TIME = 2                                         # ->duree d'attente pour le waitKey() associe a l'imshow()
iOUTPUT_NB_LINES = 100                                  # ->dimension de la sortie du DNN
iOUTPUT_NB_COLS = 7                                     # ->dimension de la sortie du DNN

#.....................................................
# liste des classes prises en compte dans ce modele: :
#..................................................... 
lstszClassName = ("background","aeroplane", "bicycle", "bird", "boat","bottle", "bus", "car", "cat", "chair","cow", "diningtable", "dog", "horse","motorbike", "person", "pottedplant","sheep", "sofa", "train", "tvmonitor")

#..................................
# couleurs pour affichage graphique
#..................................
tpColors = ((0,0,255), (0,255,0), (0,255,255),(255,0,0),(255,0,255),(255,255,0),(255,255,255))

#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# fonction de coloriage... 
# IN :  
# img  = l'image a format np/OpenCV ou faire les affichages
# lstClassID = liste des ID de classe des objets 
# lstLabels = liste des noms (chaines de caracteres) des objets
# listLocs = liste des coordonnees des rectangles englobant les objets
#            ces coordonnees sont sous forme de liste [xtl,ytl,xrd,yrd]
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
def DecorateImage( img, lstClassID, lstLabels, lstLocs):
    iNbColors = len(tpColors)
    for i in range(len(lstLabels)):
        ClassColor = tpColors[lstClassID[i] % iNbColors]
        # boite englobante
        x1 = int(lstLocs[i][0])
        y1 = int(lstLocs[i][1])
        x2 = int(lstLocs[i][2])
        y2 = int(lstLocs[i][3])
        cv2.rectangle(img, (x1,y1,x2-x1,y2-y1), ClassColor,1)
        # nom de la classe
        cv2.putText(img, lstLabels[i], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, ClassColor, 2)






#ALEX

class dnn_bottle_finder:
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
        self.model = cv2.dnn.readNet(model=szModelName, config=szModelConfig, framework='Caffe')
    
    def callbackCamera(self, data):
        # Récupération des données de la caméra
        self.camera_info = data

    def callbackDepth(self,data):
        try:
            #Recuperation image de profondeur (Alignée), Conveersion au format OpenCV
            self.depth_map = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

    def callbackColor(self,data):
        try:
            #Recuperation image RGB, Conversion au format OpenCV
            self.color_map = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #Recherche des bouteilles
            self.computeBottles(self.color_map)
        except CvBridgeError as e:
            print(e)

    def computeBottles(self, color_image):
        imgSrc = color_image
        # calcul des blobs : ATTENTION, il faut mettre l'image a la taille attendue par le DNN : 
        imgResized = cv2.resize(imgSrc,(iNET_INPUT_WIDTH, iNET_INPUT_HEIGHT), interpolation = cv2.INTER_AREA)
        blob = cv2.dnn.blobFromImage(image=imgResized, scalefactor=dbScaleFactor, size=(iNET_INPUT_WIDTH, iNET_INPUT_HEIGHT), mean=dbMeanVal)
        # necessaire pour recuperer les positions dans l'image initiale (avant redimensionnement)
        iImgWidth = imgSrc.shape[1]
        iImgHeight = imgSrc.shape[0]
        # on les "blobs" de l'image a l'entree du classifieur 
        self.model.setInput(blob)
        # declenchement de l'inference
        outputs = self.model.forward()
        final_outputs = outputs[0]
        # remise en forme des sorties
        final_outputs = final_outputs.reshape(iOUTPUT_NB_LINES, iOUTPUT_NB_COLS)
        #....................................................................
        # affichage des infos relatives aux objets detectes, le cas echeant : 
        # ................................................................... 
        iSize = final_outputs.shape
        lstLabels = []
        lstNames = []
        lstLocalisation = []
        lstProba = []
        for i in range(iSize[0]):
            # parcours des lignes
            vLine = final_outputs[i,:]
            # quelque chose ?
            if vLine[iCLASS_CONFIDENCE] > 0.5 and lstszClassName[int(vLine[iCLASS_ID])] == "bottle":
                # oui : 

                #AJOUT FILTRE ORANGE OU NOIRE!






                iClass = int(vLine[iCLASS_ID])
                lstLabels.append(iClass)
                lstProba.append(vLine[iCLASS_CONFIDENCE])
                lstNames.append(lstszClassName[iClass])
                x1 = round(vLine[iX_TOPLEFT] * iImgWidth   )
                y1 = round(vLine[iY_TOPLEFT] * iImgHeight  )
                x2 = round(vLine[iX_LOWRIGHT] * iImgWidth  )
                y2 = round(vLine[iY_LOWRIGHT] * iImgHeight )
                lstLocalisation.append([x1,y1,x2,y2])
        # affichage texte des informations de detection : 
        print("objets detectes = ")
        print(lstNames)
        print("localisations = ")
        print(lstLocalisation)
        print("probabilites reseau = ")
        print(lstProba)


        #ALEX
        for bbox_bottle in lstLocalisation:
            x_cnt_bottle = int(bbox_bottle[0] + bbox_bottle[2] // 2)
            y_cnt_bottle = int(bbox_bottle[1] + bbox_bottle[3] // 2)
            center_bottle = (x_cnt_bottle, y_cnt_bottle)

            #Display centre bouteille
            # radius = 10
            # color = (255, 0, 0)
            # thickness = 2
            # image = cv2.circle(img_result, center_bottle, radius, color, thickness)

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


        # affichage des elements detectes dans la fenetre graphique
        DecorateImage( imgSrc, lstLabels, lstNames, lstLocalisation)
        cv2.imshow('DETECTION', imgSrc)
        cKey = cv2.waitKey(iSLEEP_TIME)
        if cKey == ord(' '):
            print("FIN du traitement DNN")
            cv2.destroyAllWindows()

def main(args):
    dbf = dnn_bottle_finder()
    rospy.init_node('dnn', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    print("FIN du traitement DNN")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

#FIN ALEX











