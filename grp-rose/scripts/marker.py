#!/usr/bin/python3

#Import dependecies
import rospy
import numpy as np
import rospy, tf

#Import msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

topic = '/bottle'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('marker')

tfListener = tf.TransformListener()

markerArray = MarkerArray()

SAME_BOTTLE_RADIUS = 0.25
DIST_MIN_BETWEEN_BOTTLES = 0.3
python_marker_array = []
def compute_dist_between_two_markers(marker1, marker2):
    p1 = np.array([marker1.pose.position.x, marker1.pose.position.y, marker1.pose.position.z])
    p2 = np.array([marker2.pose.position.x, marker2.pose.position.y, marker2.pose.position.z])
    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)
    return dist


def callback(data):

    convert_coords = tfListener.transformPose("/odom", data) # on converti les coordonnées des bouteilles

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = convert_coords.pose.position.x
    marker.pose.position.y = convert_coords.pose.position.y
    marker.pose.position.z = convert_coords.pose.position.z

    

    if len(python_marker_array) == 0:
        python_marker_array.append(marker)
        markerArray.markers.append(marker)
    else:
        dist1 = compute_dist_between_two_markers(marker, python_marker_array[-1])
        if len(python_marker_array) > 1:
            for i in range(len(python_marker_array)-1):
                dist2 = compute_dist_between_two_markers(marker, python_marker_array[i])
                if dist1 > DIST_MIN_BETWEEN_BOTTLES and dist2 > SAME_BOTTLE_RADIUS:
                    python_marker_array.append(marker) 
                    markerArray.markers.append(marker)
        else:
            if dist1 > DIST_MIN_BETWEEN_BOTTLES:
                python_marker_array.append(marker) 
                markerArray.markers.append(marker)

            
    
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    publisher.publish(markerArray)

sub = rospy.Subscriber("/coord_bottle", PoseStamped, callback)

# spin() enter the program in a infinite loop
rospy.spin()