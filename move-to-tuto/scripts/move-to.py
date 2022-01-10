#!/usr/bin/python3

#Import dependecies
import rospy, tf

#Import msg
from geometry_msgs.msg import PoseStamped

rospy.init_node('move-to', anonymous=True)

tfListener = tf.TransformListener()

def callbackSub(data):
    print("DATA" + str(data))
    local_goal = tfListener.transformPose("/base_footprint", data)
    print("LOCALGOAL" + str(local_goal))

    ## Call pub velocity robot
    ## Call frequently

def callbackPub(data):
    print("callback")

    ## calcul velocity

#Déclarations subscriber
sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackSub)
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, callbackPub)

# spin() enter the program in a infinite loop
print("Start moving")
rospy.spin()