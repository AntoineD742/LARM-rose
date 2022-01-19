#!/usr/bin/python3

#Import dependecies
import rospy
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, pow, sqrt

#Import msg
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

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

        self.cmd.angular.z = 0.2 * turn

        if self.move_forward == True:
            # speed reduction when dist decrease
            if dist > 3.0:
                dist = 0.5
            elif dist > 2.0:
                dist = 0.4
            elif dist > 1.0:
                dist = 0.3
            elif dist > 0.5:
                dist = 0.2
            elif dist > 0.2:
                dist = 0.1
            elif dist > 0.05:
                dist = 0.05
            else:
                dist = 0.0
                self.cmd.angular.z = 0.0

            self.cmd.linear.x = dist

        self.commandPublisher.publish(self.cmd)

        return True

rospy.init_node('move-to-and-safelly', anonymous=True)
move_robot_to= MoveRobotTo()
rospy.spin()
