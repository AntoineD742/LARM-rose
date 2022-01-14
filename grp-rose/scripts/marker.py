#!/usr/bin/python3

#Import dependecies
import rospy, tf

#Import msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

topic = '/bottle'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('marker')

tfListener = tf.TransformListener()

markerArray = MarkerArray()

def callback(data):
    rospy.loginfo("data avant %s", data)
    local_goal = tfListener.transformPose("/odom", data)
    rospy.loginfo("data APRES %s", local_goal)

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0 # toujours laisser à 1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = local_goal.pose.position.x
    marker.pose.position.y = local_goal.pose.position.y
    marker.pose.position.z = local_goal.pose.position.z

    marker.lifetime.secs, marker.lifetime.nsecs = [0, 0]
    markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1

    publisher.publish(markerArray)

sub = rospy.Subscriber("/coord_bottle", PoseStamped, callback)

# spin() enter the program in a infinite loop
rospy.spin()