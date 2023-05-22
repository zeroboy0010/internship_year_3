
import rospy
import odrive
from odrive.enums import *
import time
from sensor_msgs.msg import JointState

my_drive = odrive.find_any()
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(2)
my_drive.axis0.controller.config.input_filter_bandwidth = 2.0
my_drive.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
my_drive.axis0.controller.config.vel_limit = 35
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
time.sleep(2)

def callback(msg):
    #rospy.loginfo(str(msg.position[1]))
    my_drive.axis0.controller.input_pos = (9/3.14)*float(msg.position[1])
    rospy.loginfo(str(float(my_drive.axis0.encoder.pos_estimate)))


def listener():
    rospy.init_node('joint_sub')
    rospy.Subscriber('joint_states',JointState,callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
