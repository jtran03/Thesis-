#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from zlac8015d import Controller

# Global variables
WHEEL_RADIUS = 0.1715 #m
WHEEL_DISTANCE = 0.4461 #m
MAX_RPM = 100
ACCEL_TIME = 200 #ms
DEACCEL_TIME = 200 #ms
VELOCITY_MODE = 3

# Call back ######################################
def zlac8015d_rpm_callback(msg):

	# Convert twist to wheel RPM
	# left_rpm = (msg.linear.x - WHEEL_DISTANCE*msg.angular.z)/(2*WHEEL_RADIUS)
	# right_rpm = (msg.linear.x + WHEEL_DISTANCE*msg.angular.z)/(2*WHEEL_RADIUS)
	left_rpm = msg.angular.x
	right_rpm = msg.angular.y
	rospy.loginfo("Received: LeftRPM: %s, RightRPM: %s", left_rpm, left_rpm)


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
	zlc.set_rpm(int(round(left_rpm)), -int(round(right_rpm)))

# Defining main  ######################################
if __name__ == '__main__':

	# Initialise ROS
	rospy.init_node('zlac8015d_node', anonymous=True)
	rospy.loginfo("Starting ZLAC8015D node")

	# Initialise publishers
	encoder_pub = rospy.Publisher("/zlac8015d/encoder", Float32MultiArray, queue_size=10)

	# Initialise publisher messages
	encoder_pub_msg = Float32MultiArray()

	# Initialise subscribers
	rospy.Subscriber("/zlac8015d/rpm", Twist, zlac8015d_rpm_callback)
	#rospy.Subscriber("/turtle1/cmd_vel", Twist, zlac8015d_wheels_rpm_callback)

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
	rate = rospy.Rate(100) # 10hz

	# Check if ros is running
	while not rospy.is_shutdown():

		leftEncoderValue, rightEncoderValue = zlc.get_encoder()
		leftRPM, rightRPM = zlc.get_rpm()

		encoder_pub_msg.data = [leftEncoderValue, rightEncoderValue, leftRPM, rightRPM]
		encoder_pub.publish(encoder_pub_msg)

		# # Grab encoder change values
		# encoder_change_pub.data = [leftEncoderValue - prevLeftEncoderValue, rightEncoderValue - prevRightEncoderValue]
		# encoder_change_pub.publish(encoder_change_pub_msg)
		# prevLeftEncoderValue = leftEncoderValue
		# prevRightEncoderValue = rightEncoderValue

		# Sleep for next rate
		rate.sleep()

	# Turn off motor
	rospy.loginfo("Shutting Off Motors")
	zlc.disable_motor()
