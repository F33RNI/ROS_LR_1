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
from time import sleep
import os
from os import listdir
from os.path import isfile, join
import signal

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, String, Int16
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# --- SETUP ---
ANGULAR_SPEED_OBSTACLE_ESCAPE = 0.5
LINEAR_SPEED = 0.14
LINEAR_SPEED_TURN = 0.02
ANGULAR_SPEED_TURN_MAX = 0.8
ANGULAR_TURN_P_GAIN = 2.6
HEADING_ALIGNMENT_SPEED = 0.4


# Publishers for debug logs, angular_z and move
log_publisher = rospy.Publisher('mission_logs', String, queue_size=8)
linear_velocity_publisher = rospy.Publisher('linear_velocity', Float64, queue_size=1)
angular_velocity_publisher = rospy.Publisher('angular_velocity', Float64, queue_size=1)
move_publisher = rospy.Publisher('move', Bool, queue_size=1)
line_use_publisher = rospy.Publisher('line_use', Int16, queue_size=1)

# OpenCV bridge for camera image
cvBridge = CvBridge()

# Velocities
linear_velocity_line = 0.
angular_velocity_line = 0.

# Lines detected flag
is_lines_detected = False

# Obstacles flags
front_obstacle = False
left_obstacle = False
right_obstacle = False

# List of detected signs
detected_signs = []

# Stage of the mision
mission_stage = 0

# List of debug messages
debug_messages = []

# Robot odometry position
odom_x = 0
odom_y = 0
odom_yaw = 0

# Target angle for turn
odom_yaw_target = 0

# Stage to switch after completing the turn
stage_after_turn = 0

# 0 - first, 1 - second
parking_zone = 0

# Line usage
LINES_ALLOW_BOTH = 3
LINES_LEFT_ONLY = 1
LINES_RIGHT_ONLY = 2
LINES_NOTHING = 0

# Mission stages
STAGE_END = -2
STAGE_FAILED = -1
STAGE_LOCATE = 0
STAGE_MOVE_TO_PARKING = 1
STAGE_RED_LIGHT = 2
STAGE_FIND_BIBOT = 3
STAGE_TURN_ODOM = 4
STAGE_ENTER_PARKING = 5
STAGE_WAIT = 6
STAGE_EXIT_PARKING = 7
STAGE_MOVE_TO_BARRIER = 8
STAGE_BARRIER_H = 9
STAGE_MAZE_1 = 10
STAGE_MAZE_2 = 11

# Find bibot spot
ODOM_FIND_BIBOT_X = 1.63
ODOM_FIND_BIBOT_Y = 1.1

# Parking steps
ODOM_PARKING_1_START = 1.30
ODOM_PARKING_1_END = 1.1
ODOM_PARKING_2_START = 0.785

ODOM_PARKING_1_X = 1.22
ODOM_PARKING_2_X = 0.65

ODOM_PARKING_ENTER_TO_Y = 1.5
ODOM_PARKING_EXIT_TO_Y = 1.2

ODOM_MAZE_TURN_X = -1.78
ODOM_MAZE_TURN_Y = -1.8

ODOM_MISSION_ACCOMPLISHED_X = 0.24
ODOM_MISSION_ACCOMPLISHED_Y = -1.78



def _map(x, in_min, in_max, out_min, out_max):
	#
	# Re-maps a number from one range to another.
	# That is, a value of fromLow would get mapped to toLow,
	# a value of fromHigh to toHigh, values in-between to values in-between, etc.
	#
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def linear_velocity_line_callback(data):
	"""
	Callback for line movement
	"""
	global linear_velocity_line
	linear_velocity_line = data.data
	if is_lines_detected:
		main_loop()


def angular_velocity_line_callback(data):
	"""
	Callback for line angular velocity
	"""
	global angular_velocity_line
	angular_velocity_line = data.data


def is_lines_detected_callback(data):
	"""
	Callback for is_lines_detected flag
	"""
	global is_lines_detected
	is_lines_detected = data.data


def front_obstacle_callback(data):
	"""
	Callback for front_obstacled flag
	"""
	global front_obstacle
	front_obstacle = data.data


def left_obstacle_callback(data):
	"""
	Callback for left_obstacled flag
	"""
	global left_obstacle
	left_obstacle = data.data


def right_obstacle_callback(data):
	"""
	Callback for right_obstacled flag
	"""
	global right_obstacle
	right_obstacle = data.data


def detected_signs_callback(data):
	"""
	Callback for array of detected signs
	"""
	global detected_signs
	detected_signs = data.data.split(',')

def main_loop():
	"""
	Main handler
	"""
	global mission_stage
	global front_obstacle, left_obstacle, right_obstacle
	global odom_x, odom_y, odom_yaw, odom_yaw_target
	global stage_after_turn, parking_zone

	# Line checking stage. Check if at least one line is visible
	if mission_stage == STAGE_LOCATE:
		# Exit if no lines detected
		if not is_lines_detected or front_obstacle:
			mission_stage = STAGE_MOVE_TO_PARKING
			wasted('Can\'t move!')
			return

		# Switch to STAGE_MOVE and start moving
		else:
			move_publisher.publish(True)
			mission_stage = STAGE_MOVE_TO_PARKING

	# Moving stage (first stage)
	elif mission_stage == STAGE_MOVE_TO_PARKING:
		# Red light
		if 'RED' in detected_signs:
			print_debug('RED LIGHT. Switching to STAGE_RED_LIGHT')
			print_debug('Waiting for green light...')
			mission_stage = STAGE_RED_LIGHT
			return

		# Red light
		if 'YELLOW' in detected_signs:
			print_debug('YELLOW LIGHT. Slowing down...')

		# Bibot in first zone
		if 'PARKING' in detected_signs and 'BIBOT' in detected_signs:
			print_debug('BIBOT detected in first zone!')
			parking_zone = 1

		# Find bibot spot
		if abs(odom_x - ODOM_FIND_BIBOT_X) <= 0.1 and abs(odom_y - ODOM_FIND_BIBOT_Y) <= 0.1:
		#if 'PARKING' in detected_signs:
			print_debug('Bibot finding spot reached. Switching to STAGE_TURN_ODOM')

			# Switch to STAGE_TURN_ODOM
			odom_yaw_target = math.radians(180)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_FIND_BIBOT
			return

		# Print debug message
		print_debug('Line correction only. L:' + str(round(linear_velocity_line, 2)) + ' A:' + str(round(angular_velocity_line, 2)))

		# Send velocities
		move_publisher.publish(True)
		linear_velocity_publisher.publish(.1 if 'YELLOW' in detected_signs else linear_velocity_line)
		angular_velocity_publisher.publish(angular_velocity_line)

	# RED light stage (waiting for vertical barrier or green light)
	elif mission_stage == STAGE_RED_LIGHT:
		# Move the robot super slowly
		move_publisher.publish(True)
		linear_velocity_publisher.publish(.01)
		angular_velocity_publisher.publish(0.)

		# Wait for green light
		if 'GREEN' in detected_signs:
			print_debug('GREEN LIGHT. Switching to STAGE_MOVE_TO_PARKING')
			mission_stage = STAGE_MOVE_TO_PARKING
			return

	# Finding parking space
	elif mission_stage == STAGE_FIND_BIBOT:

		#if ODOM_PARKING_1_END < odom_x < ODOM_PARKING_1_START and right_obstacle:
		#	parking_zone = 1

		if odom_x <= ODOM_PARKING_1_X and parking_zone == 0:
			print_debug('Parking to 1st zone...')
			odom_yaw_target = math.radians(90)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_ENTER_PARKING
			return


		if odom_x <= ODOM_PARKING_2_X and parking_zone == 1:
			print_debug('Parking to 2nd zone...')
			odom_yaw_target = math.radians(90)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_ENTER_PARKING
			return





		# Allow only left line
		line_use_publisher.publish(LINES_LEFT_ONLY)

		# Send velocities
		move_publisher.publish(True)
		linear_velocity_publisher.publish(linear_velocity_line * 0.8)
		angular_velocity_publisher.publish(angular_velocity_line)


	elif mission_stage == STAGE_ENTER_PARKING:
		if odom_y >= ODOM_PARKING_ENTER_TO_Y:
			print_debug('Parking entered. Rotating...')
			odom_yaw_target = math.radians(-90)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_WAIT
			return

		else:
			linear_velocity_publisher.publish(LINEAR_SPEED)
			rotation_controller(math.radians(90))



	elif mission_stage == STAGE_WAIT:
		print_debug('Waiting for 3 seconds...')
		rospy.sleep(3)
		print_debug('Start exiting from parking...')
		mission_stage = STAGE_EXIT_PARKING
		return



	elif mission_stage == STAGE_EXIT_PARKING:
		if odom_y <= ODOM_PARKING_EXIT_TO_Y:
			print_debug('Parking exited. Rotating...')
			odom_yaw_target = math.radians(180)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_MOVE_TO_BARRIER

			# Allow both lines
			line_use_publisher.publish(LINES_ALLOW_BOTH)
			return

		else:
			linear_velocity_publisher.publish(LINEAR_SPEED)
			rotation_controller(math.radians(-90))


	elif mission_stage == STAGE_MOVE_TO_BARRIER:

		if 'BARRIER_H' in detected_signs:
			print_debug('BARRIER_H detected! Waiting for TONNEL...')
			linear_velocity_publisher.publish(0.)
			angular_velocity_publisher.publish(0.)
			mission_stage = STAGE_BARRIER_H
			return


		# Print debug message
		print_debug('Line correction only. L:' + str(round(linear_velocity_line, 2)) + ' A:' + str(round(angular_velocity_line, 2)))

		# Send velocities
		move_publisher.publish(True)
		linear_velocity_publisher.publish(.1 if 'YELLOW' in detected_signs else linear_velocity_line)
		angular_velocity_publisher.publish(angular_velocity_line)


	elif mission_stage == STAGE_BARRIER_H:

		if 'TONNEL' in detected_signs:
			print_debug('TONNEL detected! Starting alignment...')
			odom_yaw_target = math.radians(-90)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_MAZE_1
			return

		linear_velocity_publisher.publish(0.07)
		rotation_controller(math.radians(-90))



	elif mission_stage == STAGE_MAZE_1:
		if abs(odom_x - ODOM_MAZE_TURN_X) < 0.1 and abs(odom_y - ODOM_MAZE_TURN_Y) < 0.1:
			print_debug('Maze wall 1 finished. Starting rotating...')
			odom_yaw_target = math.radians(0)
			mission_stage = STAGE_TURN_ODOM
			stage_after_turn = STAGE_MAZE_2
			return



		linear_velocity_publisher.publish(LINEAR_SPEED)
		rotation_controller(math.radians(-90))


	elif mission_stage == STAGE_MAZE_2:
		if abs(odom_x - ODOM_MISSION_ACCOMPLISHED_X) < 0.1 and abs(odom_y - ODOM_MISSION_ACCOMPLISHED_Y) < 0.1:
			print_debug('Mission accomplished!')
			mission_stage = STAGE_END
			return


		linear_velocity_publisher.publish(LINEAR_SPEED)
		rotation_controller(math.radians(0))




	elif mission_stage == STAGE_TURN_ODOM:
		# Turn finished
		if abs(odom_yaw - odom_yaw_target) < math.radians(10):
			print_debug('Turn finished. Switching to next stage')

			# Stop robot
			angular_velocity_publisher.publish(0.)
			linear_velocity_publisher.publish(0.)

			# Switch to next stage
			mission_stage = stage_after_turn
			return

		# Turning
		else:
			linear_velocity_publisher.publish(LINEAR_SPEED_TURN)
			rotation_controller(odom_yaw_target)


	


def rotation_controller(target_radians):
	global angular_velocity_publisher, odom_yaw

	pid_error = target_radians - odom_yaw
	if pid_error >= math.radians(180):
		pid_error -= math.radians(360)

	angular_velocity = ANGULAR_TURN_P_GAIN * pid_error
	if angular_velocity > ANGULAR_SPEED_TURN_MAX:
		angular_velocity = ANGULAR_SPEED_TURN_MAX
	elif angular_velocity < -ANGULAR_SPEED_TURN_MAX:
		angular_velocity = -ANGULAR_SPEED_TURN_MAX

	angular_velocity_publisher.publish(angular_velocity)


def wasted(error):
	"""
	Ends mission, prints error message and kills current thread
	"""
	move_publisher.publish(False)
	print_debug('Mission failed! ' + error)
	os.system('kill %d' % os.getpid())
	exit(0)


def mission_end():
	"""
	Ends mission and kills current thread
	"""
	move_publisher.publish(False)
	print_debug('Mission accomplished!')
	os.system('kill %d' % os.getpid())
	exit(0)


def print_debug(message):
	"""
	Pulblishes text to the debug log_publisher
	"""
	log_publisher.publish(message)


def odom_callback(data):
	global odom_x, odom_y, odom_yaw
	#curr_time = data.header.stamp
	pose = data.pose.pose

	# Caculate x, y and yaw angle of the robot
	odom_x = pose.position.x
	odom_y = pose.position.y
	(_, _, odom_yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	rospy.init_node('mission_handler')

	# Wait 5 seconds before initialization
	rospy.sleep(5)

	# Allow both lines
	line_use_publisher.publish(LINES_ALLOW_BOTH)

	# Connect subscribers for velocities and move flag
	rospy.Subscriber('linear_velocity_line', Float64, linear_velocity_line_callback, queue_size = 1)
	rospy.Subscriber('angular_velocity_line', Float64, angular_velocity_line_callback, queue_size = 1)
	rospy.Subscriber('is_lines_detected', Bool, is_lines_detected_callback, queue_size=1)

	# Connect subscribers for obstacles
	rospy.Subscriber('front_obstacle', Bool, front_obstacle_callback, queue_size=1)
	rospy.Subscriber('left_obstacle', Bool, left_obstacle_callback, queue_size=1)
	rospy.Subscriber('right_obstacle', Bool, right_obstacle_callback, queue_size=1)

	# Connect substriber for detected signs
	rospy.Subscriber('detected_signs', String, detected_signs_callback, queue_size=1)

	# Create subscriber for odometry
	rospy.Subscriber('odom', Odometry, odom_callback, queue_size=1)

	# Wait 1 second for initialization
	rospy.sleep(1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			if not is_lines_detected:
				main_loop()
				rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

	# Exit message
	print('mission_handler finished')
