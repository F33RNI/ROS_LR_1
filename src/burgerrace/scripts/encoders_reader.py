#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.
In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
For more information, please refer to <https://unlicense.org>
"""

import math

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# Robot wheel radius
WHEEL_RADIUS = 0.033

# Robot turning radius (distance between wheels / 2)
TURNING_RADIUS = 0.08

# Variable for storing previous time in seconds (to calculate time diffrerence)
last_time = 0.

# Absolute calculated X position
position_x = 0.

# Absolute calculated Y position
position_y = 0.

# Absolute calculated Yaw position (aka delta_theta accumulator)
position_yaw = 0.

is_encoders_enabled = False

# Publishers for positions
position_x_publisher = rospy.Publisher('position_x', Float64, queue_size=1)
position_y_publisher = rospy.Publisher('position_y', Float64, queue_size=1)
position_yaw_publisher = rospy.Publisher('position_yaw', Float64, queue_size=1)

# Defined types for set_absolute_positions_callback (just to not make 3 separate methods)
ENCODERS_TYPE_X = 0
ENCODERS_TYPE_Y = 1
ENCODERS_TYPE_YAW = 2

def velocity_callback(data):
	"""
	Calculates X, Y, Yaw positions (this method is called every time the velocity changes)
	"""
	global last_time, position_x, position_y, position_yaw, is_encoders_enabled

	if is_encoders_enabled:
		# Get current time in seconds
		time_now = rospy.Time.now().to_sec()

		# Calculate time difference
		time_diff = time_now - last_time

		# Check if time has passed
		if time_diff > 0.:

			# Store current time for next cycle
			last_time = time_now

			# Get linear and angular velocities from subscriber
			linear_velocity  = data.linear.x
			angular_velocity = data.angular.z

			# Calculate instantaneous velocities for both wheels
			# w = (L + (A * Tr)) / Wr
			total_velocity_left = linear_velocity - (angular_velocity * TURNING_RADIUS) / WHEEL_RADIUS
			total_velocity_right = linear_velocity + (angular_velocity * TURNING_RADIUS) / WHEEL_RADIUS

			# Calculate instantaneous distance
			# d = w * dt
			distance_left_now = total_velocity_left * time_diff
			distance_right_now = total_velocity_right * time_diff

			# Calculate delta s and delta theta (for pose estimation)
			delta_s = WHEEL_RADIUS * (distance_right_now + distance_left_now) / 2.
			delta_theta = WHEEL_RADIUS * (distance_right_now - distance_left_now) / (TURNING_RADIUS * 2.)

			# Calculate absolute positions
			position_x += delta_s * math.cos(position_yaw + (delta_theta / 2.))
			position_y += delta_s * math.sin(position_yaw + (delta_theta / 2.))
			position_yaw += delta_theta

			# Publish positions
			position_x_publisher.publish(position_x)
			position_y_publisher.publish(position_y)
			position_yaw_publisher.publish(position_yaw % math.radians(360.))

def set_absolute_positions_callback(data, type):
	"""
	Sets absolute positions
	"""
	global position_x, position_y, position_yaw
	if type == ENCODERS_TYPE_X:
		position_x = data.data

	elif type == ENCODERS_TYPE_Y:
		position_y = data.data

	elif type == ENCODERS_TYPE_YAW:
		position_yaw = data.data

def encoders_enabled_callback(data):
	"""
	Enables or disables encoders calculations
	"""
	global is_encoders_enabled
	is_encoders_enabled = data.data


# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	rospy.init_node('encoders_reader')

	# Create subscriber for velocity
	rospy.Subscriber('cmd_vel', Twist, velocity_callback, queue_size=1)

	# Create subscriber for manually defining absolute positions
	rospy.Subscriber('set_absolute_position_x', Float64, set_absolute_positions_callback, ENCODERS_TYPE_X, queue_size=1)
	rospy.Subscriber('set_absolute_position_y', Float64, set_absolute_positions_callback, ENCODERS_TYPE_Y, queue_size=1)
	rospy.Subscriber('set_absolute_position_yaw', Float64, set_absolute_positions_callback, ENCODERS_TYPE_YAW, queue_size=1)

	# Create subscriber for allow encoders calculations
	rospy.Subscriber('encoders_enabled', Bool, encoders_enabled_callback, queue_size=1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

	# Exit message
	print('encoders_reader finished')
