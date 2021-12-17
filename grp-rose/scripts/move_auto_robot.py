#!/usr/bin/python3

#Import dependecies
import math, rospy, random

#Import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MIN_DISTANCE_X = 0.5
MIN_DISTANCE_Y = 0.2
ERROR_RANGE = 0.1
ANGLE_DE_VUE = 150
TPS_QUART_DE_TOUR = 30
# Initialize ROS::node
rospy.init_node('move', anonymous=True)

firstMoveOrder = True
timeToTurn = TPS_QUART_DE_TOUR
ordre = -1
lastOrder = -2
def callback(data):
    global firstMoveOrder
    global timeToTurn
    global ordre
    global lastOrder
    #print(firstMoveOrder)
    if firstMoveOrder:
        ordre = calculObstacles(data)
        lastOrder = ordre
        firstMoveOrder = False
    #ordre = calculObstacles(data)
    if ordre == 0:
        print("j'avance")
        cmd.linear.x = 0.1  
        cmd.angular.z = 0
        lastOrder = ordre
        ordre = calculObstacles(data)
    elif ordre == 1:
        if timeToTurn > 0: #Le robot tourne
            print("obstacle à gauche, tournez à droite")
            cmd.linear.x = 0
            cmd.angular.z = -0.5
            timeToTurn -=1
        else: #Le robot a fini de tourner
            lastOrder = ordre
            ordre = calculObstacles(data)
            if (ordre == 2):
                print("Demi-tour")
                timeToTurn = 2*TPS_QUART_DE_TOUR
            else:
                timeToTurn = TPS_QUART_DE_TOUR
            print("fin de rotation")
    elif ordre == 2:
        if timeToTurn > 0: #Le robot tourne
            print("obstacle à droite, tournez à gauche")
            cmd.linear.x = 0
            cmd.angular.z = +0.5
            timeToTurn -=1
        else: #Le robot a fini de tourner
            lastOrder = ordre
            ordre = calculObstacles(data)
            if (ordre == 1):
                print("Demi-tour")
                timeToTurn = 2*TPS_QUART_DE_TOUR
            else:
                timeToTurn = TPS_QUART_DE_TOUR
            print("fin de rotation")
    print("Ordre :" + str(ordre)+ "Dernier ordre: " + str(ordre))
    commandPublisher.publish(cmd)


#def callback(data):
    #min_range = min(data.ranges[round((len(data.ranges)/2)-ANGLE_DE_VUE):round((len(data.ranges)/2)+ANGLE_DE_VUE)])
    #print("Data ranges len: " + str(len(data.ranges)))
    #min_range = data.ranges[150]
    #print("min range: " +str(min_range))
    #if (min_range < MIN_DISTANCE):
        #cmd.linear.x = 0.0
        #print("je tourne")
        #if data.header.seq%(2*TPS_QUART_DE_TOUR) < TPS_QUART_DE_TOUR:
            #cmd.angular.z = -0.0
        #else:
            #cmd.angular.z = 0.0
    #else:
        #print("j'avance")
        #cmd.linear.x = 0.0
        #cmd.angular.z = 0.0
    #calculDistance(data)   
    #commandPublisher.publish(cmd)

def calculObstacles(data):
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
    #for obstacle in obstacles[round((len(obstacles)/2)-ANGLE_DE_VUE):round((len(obstacles)/2)+ANGLE_DE_VUE)]:
    for obstacle in obstacles:
        distances.append(math.sqrt(obstacle[0]**2 + obstacle[1]**2))
        index_min = distances.index(min(distances))
    #print(distances)
    if obstacles[index_min][0] < MIN_DISTANCE_X:
        if 0 <= obstacles[index_min][1] < MIN_DISTANCE_Y:
            print(obstacles[index_min])
            return 1                            # obstacle à gauche, tournez à droite
        elif -MIN_DISTANCE_Y < obstacles[index_min][1] < 0:
            print(obstacles[index_min])
            return 2                           # obstacle à droite, tournez à gauche
    print(obstacles[index_min])
    return 0                            # continuer à avancer


sub = rospy.Subscriber("/scan", LaserScan, callback)
commandPublisher = rospy.Publisher( '/cmd_vel_mux/input/navi', Twist, queue_size=10)
cmd = Twist()

# spin() enter the program in a infinite loop
print("Start moving")
rospy.spin()