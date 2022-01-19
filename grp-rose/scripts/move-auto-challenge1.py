#!/usr/bin/python3

#Import dependecies
import math, rospy, random
import cv2

#Import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

from cv_bridge import CvBridge, CvBridgeError
#CONSTANTES / PARAMETRES
VITESSE_LINEAIRE_ROBOT_MIN = 0.1
VITESSE_LINEAIRE_ROBOT_MAX = 0.6
VITESSE_ANGULAIRE_ROBOT_MIN = 0.5
VITESSE_ANGULAIRE_ROBOT_MAX = 1


MIN_DISTANCE_X = 0.25 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe X)
MIN_DISTANCE_Y = 0.2 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe Y)
AVOID_DISTANCE_X = 1.5 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe X)
AVOID_DISTANCE_Y = 0.8 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe Y)
# AVOID_DISTANCE_X = 1 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe X)
# AVOID_DISTANCE_Y = 0.8 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe Y)

OBSTACLE_PIXEL_SIZE = 2000 #Nombre de pixel à partir duquel on considère qu'un obstacle est proche de la caméra

#OLD
BEHIND_THE_ROBOT = 50 #On set la distance des objets derrière le laser à un nombre infiniment grand


#ATTENTION ON NE DETECTE PAS LES BOUTEILLES POUR L'INSTANT


class mazeRunner:
    def __init__(self):
        self.movement_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 10)   
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        self.move_command = Twist()
        self.current_forward_speed = VITESSE_LINEAIRE_ROBOT_MIN
        self.current_angular_speed = 0

        self.bridge = CvBridge()            #Conversion Images OpenCV-ROS
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callbackDepth)  #Recuperation image de profondeur (Alignée)
        self.depth_map = None   #Frame Color    
        self.depth_obstacle_ahead = False
        
        self.generate_new_direction = False

    def callbackLaser(self, data):    #Mouvement du robot selon la data reçu par le laser
        order = self.decisionMouvement(data)
        self.setFwdSpeed(order)
        self.setAngulaireSpeed(order)
        self.move_command.linear.x = self.current_forward_speed
        self.move_command.angular.z = self.current_angular_speed
        self.movement_pub.publish(self.move_command)
    
    def setFwdSpeed(self, order): 
        #0 DEMI TOUR
        #1 AVANCER TOUT DROIT (VITESSE MAX)
        #2 OBJET LOINTAIN SUR LA GAUCHE
        #3 OBJET LOINTAIN SUR LA DROITE
        #4 OBJET PROCHE SUR LA GAUCHE
        #5 OBJET PROCHE SUR LA DROITE
        if order == 0:
            #self.current_forward_speed = - VITESSE_LINEAIRE_ROBOT_MIN
            self.current_forward_speed = 0
        elif order == 1:  
            if(self.current_forward_speed < VITESSE_LINEAIRE_ROBOT_MAX):
                self.current_forward_speed += 0.05
            else:
                self.current_forward_speed = VITESSE_LINEAIRE_ROBOT_MAX
        elif order == 2 or order == 3:
            if(self.current_forward_speed > VITESSE_LINEAIRE_ROBOT_MIN):
                self.current_forward_speed -= 0.01
            else:
                self.current_forward_speed = VITESSE_LINEAIRE_ROBOT_MIN
        else:
            self.current_forward_speed = 0
        #print("Forward speed: " + str(self.current_forward_speed))
    
    def setAngulaireSpeed(self, order):
        #0 DEMI TOUR
        #1 AVANCER TOUT DROIT (VITESSE MAX)
        #2 OBJET LOINTAIN SUR LA GAUCHE
        #3 OBJET LOINTAIN SUR LA DROITE
        #4 OBJET PROCHE SUR LA GAUCHE
        #5 OBJET PROCHE SUR LA DROITE
        if order == 0:
            if self.generate_new_direction:
                if random.random() < 0.5:
                    self.current_angular_speed = +VITESSE_ANGULAIRE_ROBOT_MAX
                else:
                    self.current_angular_speed = -VITESSE_ANGULAIRE_ROBOT_MAX
                self.generate_new_direction = False

            
        elif order == 1:
            self.current_angular_speed = 0
            self.generate_new_direction = True #Prochaine fois que le robot devra faire demi-tour il generera aleatoirement une direction
        elif order == 2:
            self.current_angular_speed = -VITESSE_ANGULAIRE_ROBOT_MIN
        elif order == 3:
            self.current_angular_speed = +VITESSE_ANGULAIRE_ROBOT_MIN
        elif order == 4:
            self.current_angular_speed = -VITESSE_ANGULAIRE_ROBOT_MAX
        elif order == 5:
            self.current_angular_speed = +VITESSE_ANGULAIRE_ROBOT_MAX
        #else:
        #print("Angular speed: " + str(self.current_angular_speed))
        


    

    def decisionMouvement(self,data):    #Calcul des distances entre le robot et l'obstacle le plus proche, prise d'une décision permettant d'éviter l'obstacles
        #0 DEMI TOUR
        #1 AVANCER TOUT DROIT (VITESSE MAX)
        #2 OBJET LOINTAIN SUR LA GAUCHE
        #3 OBJET LOINTAIN SUR LA DROITE
        #4 OBJET PROCHE SUR LA GAUCHE
        #5 OBJET PROCHE SUR LA DROITE
        
        if(self.depth_obstacle_ahead):
            return 0

        #Calcul des distances
        obstacles= []
        distances = []
        angle= data.angle_min
        for aDistance in data.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= [ 
                math.cos(angle) * aDistance, 
                math.sin(angle) * aDistance
                ]
                obstacles.append(aPoint)
            angle+= data.angle_increment
        for obstacle in obstacles:
            if obstacle[0] < 0:
                distances.append(BEHIND_THE_ROBOT)
            else:
                distances.append(math.sqrt(obstacle[0]**2 + obstacle[1]**2))
            index_min = distances.index(min(distances))

        #Prise de décision
        if obstacles[index_min][0] < MIN_DISTANCE_X and obstacles[index_min][0] > 0:
            if 0 <= obstacles[index_min][1] < MIN_DISTANCE_Y:
                return 4                            # obstacle proche à gauche, tournez à droite
            elif -MIN_DISTANCE_Y < obstacles[index_min][1] < 0:
                return 5                           # obstacle proche  à droite, tournez à gauche
        elif obstacles[index_min][0] < AVOID_DISTANCE_X and obstacles[index_min][0] > 0:
            if 0 <= obstacles[index_min][1] < AVOID_DISTANCE_Y:
                return 2                            # obstacle lointain à gauche, tournez à droite
            elif -AVOID_DISTANCE_Y < obstacles[index_min][1] < 0:
                return 3                            # obstacle lointain à droite, tournez à gauche
        return 1                            # pas d'obstacle, continuer à avancer

    def callbackDepth(self,data):
        try:
            #Recuperation image de profondeur (Alignée), Conveersion au format OpenCV
            self.depth_map = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.depth_map = cv2.convertScaleAbs(self.depth_map, alpha=0.1)
            self.depth_map = cv2.inRange(self.depth_map, 1 , 40)
            self.depth_map = cv2.bitwise_not(self.depth_map)
            cv2.imshow("Depth", self.depth_map)
            cv2.waitKey(3)
            heightDepthMap = self.depth_map.shape[0]
            widthDepthMap = self.depth_map.shape[1]
            nbrPixelsDetectes = (heightDepthMap*widthDepthMap) - cv2.countNonZero(self.depth_map)
            if nbrPixelsDetectes > OBSTACLE_PIXEL_SIZE:
                self.depth_obstacle_ahead = True
            else:
                self.depth_obstacle_ahead = False
            #print(nbrPixelsDetectes)

        except CvBridgeError as e:
            print(e)
    

def main():
    mR = mazeRunner()
    rospy.init_node('move', anonymous=True)
    print("Start moving")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows() 
if __name__ == '__main__':
    main()











# #Import dependecies
# import math, rospy, random

# #Import msg
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# #DECLARATIONS CONSTANTES
# MIN_DISTANCE_X = 0.25 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe X)
# MIN_DISTANCE_Y = 0.2 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe Y)
# ANGLE_DE_VUE = 150  #Angle de vue du laser, plus ce nombre est grand plus l'angle est important
# TPS_QUART_DE_TOUR = 30 #Temps pendant lequel le robot tourne lorsqu'il rencontre un obstacle
# BEHIND_THE_ROBOT = 50 #On set la distance des objets derrière le laser à un nombre infiniment grand
# VITESSE_LINEAIRE_ROBOT = 0.3#Vitesse du robot en ligne droite
# VITESSE_ANGULAIRE_ROBOT = 0.5 #Vitesse du robot pour tourner


# #Challenge 3
# VITESSE_LINEAIRE_ROBOT_MIN = 0.3 #Vitesse min du robot en ligne droite
# VITESSE_LINEAIRE_ROBOT_MAX = 0.8 #Vitesse  max du robot en ligne droite


# #Déclarations variables globales
# firstMoveOrder = True   #Booléan d'init
# timeToTurn = TPS_QUART_DE_TOUR  #Counter qui se fait désincrementer pour tourner
# ordre = -1  # 0 = Marche avant, 1 Tourne à gauche, 2 Tourne à droite

# rospy.init_node('move', anonymous=True)


# def callbackLaser(data):    #Mouvement du robot selon la data reçu par le laser
#     global firstMoveOrder
#     global timeToTurn
#     global ordre
#     if firstMoveOrder: #Init
#         ordre = decisionMouvement(data)
#         firstMoveOrder = False

#     if ordre == 0: #Le robot avance en ligne droite
#         cmd.linear.x = VITESSE_LINEAIRE_ROBOT 
#         cmd.angular.z = 0
#         ordre = decisionMouvement(data)
#     elif ordre == 1:
#         if timeToTurn > 0: #Le robot tourne à droite
#             cmd.linear.x = 0
#             cmd.angular.z = -VITESSE_ANGULAIRE_ROBOT
#             timeToTurn -=1
#         else: #Le robot a fini de tourner
#             ordre = decisionMouvement(data)
#             if (ordre == 2): #Le robot a tourné successivement à droite et à gauche sans résultat ==> Il fait demi-tour
#                 timeToTurn = 2*TPS_QUART_DE_TOUR
#             else:
#                 timeToTurn = TPS_QUART_DE_TOUR
#     elif ordre == 2:
#         if timeToTurn > 0: #Le robot tourne à gauche
#             cmd.linear.x = 0
#             cmd.angular.z = +VITESSE_ANGULAIRE_ROBOT
#             timeToTurn -=1
#         else: #Le robot a fini de tourner
#             ordre = decisionMouvement(data)
#             if (ordre == 1):    #Le robot a tourné successivement à gauche et à droite sans résultat ==> Il fait demi-tour
#                 timeToTurn = 2*TPS_QUART_DE_TOUR
#             else:
#                 timeToTurn = TPS_QUART_DE_TOUR
#     commandPublisher.publish(cmd)

# def decisionMouvement(data):    #Calcul des distances entre le robot et l'obstacle le plus proche, prise d'une décision permettant d'éviter l'obstacles
#     #Calcul des distances
#     obstacles= []
#     distances = []
#     angle= data.angle_min
#     for aDistance in data.ranges :
#         if 0.1 < aDistance and aDistance < 5.0 :
#             aPoint= [ 
#             math.cos(angle) * aDistance, 
#             math.sin(angle) * aDistance
#             ]
#             obstacles.append(aPoint)
#         angle+= data.angle_increment
#     for obstacle in obstacles:
#         if obstacle[0] < 0:
#             distances.append(BEHIND_THE_ROBOT)
#         else:
#             distances.append(math.sqrt(obstacle[0]**2 + obstacle[1]**2))
#         index_min = distances.index(min(distances))

#     #Prise de décision
#     if obstacles[index_min][0] < MIN_DISTANCE_X and obstacles[index_min][0] > 0:
#         if 0 <= obstacles[index_min][1] < MIN_DISTANCE_Y:
#             return 1                            # obstacle à gauche, tournez à droite
#         elif -MIN_DISTANCE_Y < obstacles[index_min][1] < 0:
#             return 2                           # obstacle à droite, tournez à gauche
#     return 0                            # pas d'obstacle, continuer à avancer

# #Déclarations subscriber et publisher
# sub = rospy.Subscriber("/scan", LaserScan, callbackLaser)
# commandPublisher = rospy.Publisher( '/cmd_vel_mux/input/navi', Twist, queue_size=10)
# cmd = Twist()

# # spin() enter the program in a infinite loop
# print("Start moving")
# rospy.spin()