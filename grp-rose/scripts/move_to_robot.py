#!/usr/bin/python3

#Import dependecies
import rospy
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, pow, sqrt

#Import msg
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# CONSTANTES

COEFF_VITESSE_TURN = 0.2

DISTANCE_1 = 3.0
VITESSE_1 = 1.0

DISTANCE_2 = 2.0
VITESSE_2 = 0.8

DISTANCE_3 = 1.0
VITESSE_3 = 0.6

DISTANCE_4 = 0.5
VITESSE_4 = 0.4

DISTANCE_5 = 0.3
VITESSE_5 = 0.2

DISTANCE_6 = 0.2
VITESSE_6 = 0.1

class MoveRobotTo:

    def __init__(self):
        self.goal = None
        self.move_forward = False
        self.cmd = Twist()
        self.x = None
        self.y = None
        self.theta = None
        # Publisher
        self.commandPublisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        # Subscriber
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.update_goal)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        # Timer
        rospy.Timer(rospy.Duration(0.1), self.move_command, oneshot=False)


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


    def move_command(self, event=None):
        # If there is no goal, do nothing
        if self.goal is None:
            return False

        inc_x = self.goal.pose.position.x - self.x      # distance robot to goal in x
        inc_y = self.goal.pose.position.y - self.y      # distance robot to goal in y
        angle_to_goal = atan2(inc_y, inc_x)             # calculate angle through distance from robot to goal in x and y
        dist = sqrt(pow(inc_x, 2) + pow(inc_y, 2))      # calculate distance

        # find out which turndirection is better
        # the bigger the angle, the bigger turn, - when clockwise
        turn = atan2(sin(angle_to_goal - self.theta), cos(angle_to_goal - self.theta))

        if abs(angle_to_goal - self.theta) < 0.1:
            self.move_forward = True

        self.cmd.angular.z = COEFF_VITESSE_TURN * turn

        if self.move_forward == True:
            # speed reduction when dist decrease
            if dist > DISTANCE_1:
                speed = VITESSE_1
            elif dist > DISTANCE_2:
                speed = VITESSE_2
            elif dist > DISTANCE_3:
                speed = VITESSE_3
            elif dist > DISTANCE_4:
                speed = VITESSE_4
            elif dist > DISTANCE_5:
                speed = VITESSE_5
            elif dist > DISTANCE_6:
                speed = VITESSE_6
            else:
                speed = 0.0
                self.cmd.angular.z = 0.0

            self.cmd.linear.x = speed

        self.commandPublisher.publish(self.cmd)

        return True

rospy.init_node('move-to-and-safelly', anonymous=True)
move_robot_to= MoveRobotTo()
rospy.spin()
