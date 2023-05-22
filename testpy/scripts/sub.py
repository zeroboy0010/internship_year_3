
import rospy
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import *
import time

my_drive = odrive.find_any()
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis0.controller.config.input_mode = 2
my_drive.axis0.controller.config.control_mode = 2
time.sleep(2)

def callback(msg):
    # rospy.loginfo("Received a /cmd_vel message!")
    # rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    my_drive.axis0.controller.input_vel = msg.linear.x
    

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.loginfo(str(float(my_drive.axis0.encoder.vel_estimate)))
    rospy.spin()

if __name__ == '__main__':
    listener()
