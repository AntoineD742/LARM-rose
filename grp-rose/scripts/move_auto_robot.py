#!/usr/bin/python3

#Import dependecies
import math, rospy, random
import cv2
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, pow, sqrt

#Import msg
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

from cv_bridge import CvBridge, CvBridgeError

#CONSTANTES / PARAMETRES
VITESSE_LINEAIRE_ROBOT_MIN = 0.1
VITESSE_LINEAIRE_ROBOT_MAX = 0.3
VITESSE_ANGULAIRE_ROBOT_MIN = 0.5
VITESSE_ANGULAIRE_ROBOT_MAX = 1


MIN_DISTANCE_X = 0.25 #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe X)
MIN_DISTANCE_Y = 0.2  #Distance pour laquelle le robot considère l'obstacle comme à éviter (Sur l'axe Y)
AVOID_DISTANCE_X = 1.5 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe X)
AVOID_DISTANCE_Y = 0.8 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe Y)
# AVOID_DISTANCE_X = 1 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe X)
# AVOID_DISTANCE_Y = 0.8 #Distance pour laquelle le robot considère l'obstacle comme lointain (Sur l'axe Y)

OBSTACLE_PIXEL_SIZE = 2000 #Nombre de pixel à partir duquel on considère qu'un obstacle est proche de la caméra

#OLD
BEHIND_THE_ROBOT = 50 #On set la distance des objets derrière le laser à un nombre infiniment grand


# CONSTANTES

COEFF_VITESSE_TURN = 0.2

DISTANCE_1 = 3.0
VITESSE_1 = 0.7

DISTANCE_2 = 2.0
VITESSE_2 = 0.5

DISTANCE_3 = 1.0
VITESSE_3 = 0.4

DISTANCE_4 = 0.5
VITESSE_4 = 0.2

DISTANCE_5 = 0.3
VITESSE_5 = 0.1

DISTANCE_6 = 0.2
VITESSE_6 = 0.05


class RobotMouvement:

    def __init__(self):
        self.cmd = Twist()

        self.bridge = CvBridge()            #Conversion Images OpenCV-ROS
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callbackDepth)  #Recuperation image de profondeur (Alignée)
        self.depth_map = None   #Frame Color    
        self.depth_obstacle_ahead = False
        
        self.generate_new_direction = False

        self.goal = None
        self.move_forward = False

        self.x = None
        self.y = None
        self.theta = None
        self.dist = None

        self.current_forward_speed = 0
        self.current_angular_speed = 0

        # Publisher
        self.commandPublisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size = 10)   
        # Subscriber
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        # Timer
        #rospy.Timer(rospy.Duration(0.1), self.move_command, oneshot=False)

    def update_goal(self, goalData):
        # Record goal pose given in goalData in odom frame
        self.goal = goalData
        self.move_forward = False

    def update_pose(self, goalPose):
        # Record robot pose given in goalPose in odom frame
        self.x = goalPose.pose.pose.position.x
        self.y = goalPose.pose.pose.position.y
        rot_q = goalPose.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    
    
    
    
    def callbackLaser(self, data):    #Mouvement du robot selon la data reçu par le laser
        order = self.decisionMouvement(data)
        self.setFwdSpeed(order)
        self.setAngulaireSpeed(order)
        self.cmd.linear.x = self.current_forward_speed
        self.cmd.angular.z = self.current_angular_speed
        self.commandPublisher.publish(self.cmd)
    
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
            self.move_command_linear()
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
            self.move_command_angular()
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



    def move_command_linear(self, event=None):
        # If there is no goal, do nothing
        if self.goal is None:
            return False

        inc_x = self.goal.pose.position.x - self.x      # distance robot to goal in x
        inc_y = self.goal.pose.position.y - self.y      # distance robot to goal in y
        self.dist = sqrt(pow(inc_x, 2) + pow(inc_y, 2))      # calculate distance

        if self.move_forward == True:
            # speed reduction when dist decrease
            if self.dist > DISTANCE_1:
                self.current_forward_speed = VITESSE_1
            elif self.dist > DISTANCE_2:
                self.current_forward_speed = VITESSE_2
            elif self.dist > DISTANCE_3:
                self.current_forward_speed = VITESSE_3
            elif self.dist > DISTANCE_4:
                self.current_forward_speed = VITESSE_4
            elif self.dist > DISTANCE_5:
                self.current_forward_speed = VITESSE_5
            elif self.dist > DISTANCE_6:
                self.current_forward_speed = VITESSE_6
            else:
                self.current_forward_speed = 0.0
                self.current_angular_speed = 0.0

        return True

    def move_command_angular(self, event=None):
        # If there is no goal, do nothing
        if self.goal is None:
            return False

        inc_x = self.goal.pose.position.x - self.x      # distance robot to goal in x
        inc_y = self.goal.pose.position.y - self.y      # distance robot to goal in y
        angle_to_goal = atan2(inc_y, inc_x)             # calculate angle through distance from robot to goal in x and y

        # find out which turndirection is better
        # the bigger the angle, the bigger turn, - when clockwise
        turn = atan2(sin(angle_to_goal - self.theta), cos(angle_to_goal - self.theta))

        if abs(angle_to_goal - self.theta) < 0.1:
            self.move_forward = True

        if self.dist > DISTANCE_6:
            self.current_angular_speed = COEFF_VITESSE_TURN * turn
        else:
            self.current_angular_speed = 0.0

        return True



def main():
    rM = RobotMouvement()
    rospy.init_node('move', anonymous=True)
    print("Start moving")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows() 
if __name__ == '__main__':
    main()