#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
from zlac8015d import Controller

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
	right_rpm = (msg.linear.x + WHEEL_DISTANCE*msg.angular.z)/WHEEL_RADIUS
	#rospy.loginfo("Calcualted: LeftRPM: %s, RightRPM: %s", int(left_rpm), int(left_rpm))

	# Check if RPM is over the limit
	if (abs(int(left_rpm)) > MAX_RPM):

		# Reset RPM
		rospy.loginfo("Max Left RPM Exceeded. Resetting...")
		left_rpm = 0

	# Check if RPM is over the limit
	if (abs(int(right_rpm)) > MAX_RPM):

		# Reset RPM
		rospy.loginfo("Max Right RPM Exceeded. Resetting...")
		right_rpm = 0

	# Set the RPM
	rospy.loginfo("Got RPM. ")
	zlc.set_rpm(int(left_rpm), int(right_rpm))

# Defining main  ######################################
if __name__ == '__main__':

	# Initialise ROS
	rospy.init_node('zlac8015d_node', anonymous=True)
	rospy.loginfo("Starting ZLAC8015D node")

	# Initialise publishers
	encoder_pub = rospy.Publisher("/zlac8015d/encoder", Int32MultiArray, queue_size=10)
	RPM_pub = rospy.Publisher("/zlac8015d/measured_RPM", Int32MultiArray, queue_size=10)

	# Initialise publisher messages
	encoder_pub_msg = Int32MultiArray()
	RPM_pub_msg = Int32MultiArray()

	# Initialise subscribers
	rospy.Subscriber("/zlac8015d/wheels_rpm", Twist, zlac8015d_wheels_rpm_callback)
	rospy.Subscriber("/turtle1/cmd_vel", Twist, zlac8015d_wheels_rpm_callback)


	# Import ZLAC8015D API
	zlc = Controller()

	# Iniitalise the ZLAC8015D
	rospy.loginfo("Initialising Motor Driver")
	zlc.disable_motor()

	# Set acceleration and deceleration times
	zlc.set_accel_time(ACCEL_TIME,ACCEL_TIME)
	zlc.set_decel_time(DEACCEL_TIME,DEACCEL_TIME)

	# Default set to speed mode (wheel rpm)
	zlc.set_mode(VELOCITY_MODE)
	zlc.enable_motor()

	# Define rate as 10Hz
	rospy.loginfo("Motors successfully up and running")
	rate = rospy.Rate(40) # 10hz

	# Check if ros is running
	while not rospy.is_shutdown():

		# Grab encoder values
		leftEncoderValue, rightEncoderValue = zlc.get_encoder()
		encoder_pub_msg.data = [leftEncoderValue, rightEncoderValue]
		encoder_pub.publish(encoder_pub_msg)

		# Grab velocity values
		leftRPM, rightRPM = zlc.get_rpm()
		RPM_pub_msg.data = [leftRPM, rightRPM]
		RPM_pub.publish(RPM_pub_msg)

		# Sleep for next rate
		rate.sleep()

	# Turn off motor
	rospy.loginfo("Shutting Off Motors")
	zlc.disable_motor()