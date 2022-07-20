import rospy
from std_msgs.msg import Int32MultiArray, Int8, Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros
from zlac8015d import Controller
import time
import numpy as np

# Global variables
WHEEL_RADIUS = 0.1715 #m
WHEEL_DISTANCE = 0.5 #m
MAX_RPM = 100
ACCEL_TIME = 200 #ms
DEACCEL_TIME = 200 #ms
VELOCITY_MODE = 3

# Call back ######################################
def zlac8015d_wheels_rpm_callback(msg):

	# Convert twist to wheel RPM
	left_rpm = (msg.linear.x - WHEEL_DISTANCE*msg.angular.z)/WHEEL_RADIUS
	right_rpm = -(msg.linear.x + WHEEL_DISTANCE*msg.angular.z)/WHEEL_RADIUS

	# Check if RPM is over the limit
	if (-MAX_RPM < left_rpm < MAX_RPM):

		# Reset RPM
		rospy.loginfo("Max Left RPM Exceeded. Resetting...")
		left_rpm = 0

	# Check if RPM is over the limit
	if (-MAX_RPM < right_rpm < MAX_RPM):

		# Reset RPM
		rospy.loginfo("Max Right RPM Exceeded. Resetting...")
		right_rpm = 0

	# Set the RPM
	zlc.set_rpm(int(left_rpm), int(right_rpm))

# Defining main  ######################################
if __name__ == '__main__':

	# Initialise ROS
	rospy.init_node('zlac8015d_node', anonymous=True)
	rospy.loginfo("Start ZLAC8015D node")

	# Initialise publishers
	encoder_pub = rospy.Publisher("/zlac8015d/encoder_cmd", Float32MultiArray, queue_size=10)

	# Initialise publisher messages
	encoder_pub_msg = Int32MultiArray()

	# Initialise subscribers
	rospy.Subscriber("/zlac8015d/wheels_rpm", Twist, zlac8015d_wheels_rpm_callback)

	# Import ZLAC8015D API
	zlc = Controller()

	# Iniitalise the ZLAC8015D
	zlc.disable_motor()

	# Set acceleration and deceleration times
	zlc.set_accel_time(ACCEL_TIME,ACCEL_TIME)
	zlc.set_decel_time(DEACCEL_TIME,DEACCEL_TIME)

	# Default set to speed mode (wheel rpm)
	zlc.set_mode(VELOCITY_MODE)
	zlc.enable_motor()

	# Define rate as 10Hz
	rate = rospy.Rate(10) # 10hz

	# Check if ros is running
    while not rospy.is_shutdown():

		# Grab encoder values
		encoder_pub_msg.msg[0], encoder_pub_msg.msg[1] = zlc.get_encoder()
		encoder_pub.publish()

		# Sleep until the next rate
	    rate.sleep()

	# Disable motors if we outside loop
	zlc.disable_motor()
