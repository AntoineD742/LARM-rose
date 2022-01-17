#!/usr/bin/python3

#Import dependecies
import math, rospy, random

#Import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#DECLARATIONS CONSTANTES
MIN_DISTANCE_X = 0.5 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe X)
MIN_DISTANCE_Y = 0.4 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe Y)
ANGLE_DE_VUE = 150  #Angle de vue du laser, plus ce nombre est grand plus l'angle est important
TPS_QUART_DE_TOUR = 30 #Temps pendant lequel le robot tourne lorsqu'il rencontre un obstacle
BEHIND_THE_ROBOT = 50 #On set la distance des objets derrière le laser à un nombre infiniment grand
VITESSE_LINEAIRE_ROBOT = 1.0 #Vitesse du robot en ligne droite
VITESSE_ANGULAIRE_ROBOT = 0.5 #Vitesse du robot pour tourner

#Déclarations variables globales
firstMoveOrder = True   #Booléan d'init
timeToTurn = TPS_QUART_DE_TOUR  #Counter qui se fait désincrementer pour tourner
ordre = -1  # 0 = Marche avant, 1 Tourne à gauche, 2 Tourne à droite


rospy.init_node('move', anonymous=True) 


def callbackLaser(data):        #Mouvement du robot selon la data reçu par le laser
    global firstMoveOrder
    global timeToTurn
    global ordre
    if firstMoveOrder:  #Init
        ordre = decisionMouvement(data)
        firstMoveOrder = False

    if ordre == 0:      #Le robot avance en ligne droite
        cmd.linear.x = VITESSE_LINEAIRE_ROBOT 
        cmd.angular.z = 0
        ordre = decisionMouvement(data)
    elif ordre == 1:
        if timeToTurn > 0: #Le robot tourne à droite
            cmd.linear.x = 0
            cmd.angular.z = -VITESSE_ANGULAIRE_ROBOT
            timeToTurn -=1
        else: #Le robot a fini de tourner
            ordre = decisionMouvement(data)
            if (ordre == 2):    #Le robot a tourné successivement à droite et à gauche sans résultat ==> Il fait demi-tour
                timeToTurn = 2*TPS_QUART_DE_TOUR
            else:
                timeToTurn = TPS_QUART_DE_TOUR
    elif ordre == 2:
        if timeToTurn > 0: #Le robot tourne à gauche
            cmd.linear.x = 0
            cmd.angular.z = +VITESSE_ANGULAIRE_ROBOT
            timeToTurn -=1
        else: #Le robot a fini de tourner
            ordre = decisionMouvement(data)
            if (ordre == 1):    #Le robot a tourné successivement à gauche et à droite sans résultat ==> Il fait demi-tour
                timeToTurn = 2*TPS_QUART_DE_TOUR
            else:
                timeToTurn = TPS_QUART_DE_TOUR
    commandPublisher.publish(cmd)


def decisionMouvement(data):  #Calcul des distances entre le robot et l'obstacle le plus proche, prise d'une décision permettant d'éviter l'obstacles
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
    if obstacles[index_min][0] < MIN_DISTANCE_X and obstacles[index_min][0] > 0.05:
        if 0 <= obstacles[index_min][1] < MIN_DISTANCE_Y:
            return 1                            # obstacle à gauche, tourner à droite
        elif -MIN_DISTANCE_Y < obstacles[index_min][1] < 0:
            return 2                           # obstacle à droite, tourner à gauche
    return 0                            # pas d'obstacle, continuer à avancer


#Déclarations subscriber et publisher
sub = rospy.Subscriber("/scan", LaserScan, callbackLaser)
commandPublisher = rospy.Publisher( '/cmd_vel_mux/input/navi', Twist, queue_size=10)
cmd = Twist()
print("Start moving...")
rospy.spin()