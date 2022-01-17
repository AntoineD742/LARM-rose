#!/usr/bin/python3

#Import dependecies
import sys
import rospy, tf
from math import sqrt, pow

#Import msg
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

class MoveRobotTo:

    def __init__(self):
        self.pose = Odometry()
        self.goal= PoseStamped()
        self.cmd = Twist()
        self.tfListener = tf.TransformListener()
        # Publisher:
        self.commandPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Subscriber:
        rospy.Subscriber('/goal', PoseStamped, self.update_goal)
        rospy.Subscriber('/odom', Odometry, self.robot_pose)
        # Timer:
        rospy.Timer(rospy.Duration(0.1), self.move_command, oneshot=False)
       

    def robot_pose(self, robot_position):
        self.pose = robot_position

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.position.y - self.pose.pose.pose.position.y), 2) +
                    pow((goal_pose.pose.position.z - self.pose.pose.pose.position.z), 2))


    def update_goal(self, goalData):
        # Record goal pose given in goalData in a fixed frame from 
        self.goal = self.tfListener.transformPose("/base_footprint", goalData)

    def move_command(self, event=None):
        # Compute command toward self.goal
        goal_pose = self.goal

        while self.euclidean_distance(goal_pose) >= 0.1:
            self.cmd.linear.x = 1.0
            self.cmd.linear.y = 0.0
            self.cmd.linear.z = 0.0

            self.cmd.angular.x = goal_pose.pose.orientation.x
            self.cmd.angular.y = goal_pose.pose.orientation.y
            self.cmd.angular.z = goal_pose.pose.orientation.z

            self.commandPublisher.publish(self.cmd)
        
        self.cmd.linear.x = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        self.commandPublisher.publish(self.cmd)

rospy.init_node('move-to-and-safelly', anonymous=True)
move_robot_to= MoveRobotTo()
move_robot_to.move_command()
rospy.spin()
