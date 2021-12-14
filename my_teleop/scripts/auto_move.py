
#!/usr/bin/python3
#Import dependecies
import math, rospy
#Import msg
from geometry_msgs.msg import Twist

# Initialize ROS::node
rospy.init_node('move', anonymous=True)	#Node de nom "move"

commandPublisher = rospy.Publisher(		#Permet de relier le node "move" au topic '//cmd_vel', Message de type Twist
    '/cmd_vel',
    Twist, queue_size=10
)

# Publish velocity commandes:
def move_command(data):
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    cmd= Twist()			#Création d'un message twist
    cmd.linear.x= 0.1			#Set la valeur du vecteurs
    commandPublisher.publish(cmd)	#Le publisher envoie le message cmd (Type Twist) au topic '/cmd_vel/' (qui est relié au node 'move')

# call the move_command at a regular frequency:
rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

# spin() enter the program in a infinite loop
print("Start moving")
rospy.spin()
