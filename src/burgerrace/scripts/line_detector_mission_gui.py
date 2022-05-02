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

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, Int16

# --- PERSPECTIVE TRANSFORM ---
# X disance between bottom left corner of source frame and transformed frame
TRANSFORM_DIST_X = 125

# --- HSV MASK COLOR ---
# Yellow left line (Main)
LEFT_H_LOWER = 25
LEFT_H_UPPER = 35
LEFT_S_LOWER = 200
LEFT_S_UPPER = 255
LEFT_V_LOWER = 200
LEFT_V_UPPER = 255

# White right line (Aux)
RIGHT_H_LOWER = 0
RIGHT_H_UPPER = 180
RIGHT_S_LOWER = 0
RIGHT_S_UPPER = 20
RIGHT_V_LOWER = 200
RIGHT_V_UPPER = 255


# --- PROPORTIONAL CONTROLLER ---
# Between X and end of the lines (top of the lines)
P_TERM_CENTER_X = 0.08

# Incline between OY (Y axis) and vector of the lines
P_TERM_INCLINE_LEFT = 0.8
P_TERM_INCLINE_RIGHT = 0.8

# Between Y of the line start (bottom of the lines) and bottom of the frame
P_TERM_BOTTOM_Y_LEFT = 0.6
P_TERM_BOTTOM_Y_RIGHT = 0.4

# Publishers for debug image, angular_z and is_lines_detected to the main mission file
image_publisher = rospy.Publisher('image_line', Image, queue_size=1)
linear_velocity_publisher = rospy.Publisher('linear_velocity_line', Float64, queue_size=1)
angular_velocity_publisher = rospy.Publisher('angular_velocity_line', Float64, queue_size=1)
is_lines_detected = rospy.Publisher('is_lines_detected', Bool, queue_size=1)

# OpenCV bridge for camera image
cvBridge = CvBridge()

# Warp transformation matrix
perspective_matrix = None

# Is started flag
robot_started = False

# Line usage (0 - nothing, 1 - left, 2 - right, 3 - both)
line_use = 0

# Line usage
LINES_ALLOW_BOTH = 3
LINES_LEFT_ONLY = 1
LINES_RIGHT_ONLY = 2
LINES_NOTHING = 0

def image_callback(data):
	global perspective_matrix, line_use

	# Get source image
	frame_source = cvBridge.imgmsg_to_cv2(data, 'bgr8')

	# Get size of image
	height, width, _ = frame_source.shape

	# Define perspective matrix on first run
	if perspective_matrix is None:
		# Create perspective matrix
		transform_src = np.array([
			[0, 0],
			[width - 1, 0],
			[width - 1, height - 1],
			[0, height - 1]], dtype = 'float32')

		transform_dst = np.array([
			[0, 0],
			[width - 1, 0],
			[width - TRANSFORM_DIST_X, height - 1],
			[TRANSFORM_DIST_X, height - 1]], dtype = 'float32')
		perspective_matrix = cv2.getPerspectiveTransform(transform_src, transform_dst)

	# Fix perspective of the frame
	frame_warped = cv2.warpPerspective(frame_source, perspective_matrix, (width, height))

	# Copy warped frame
	frame_debug = frame_warped.copy()

	# Convert to HSV
	frame_hsv = cv2.cvtColor(frame_warped, cv2.COLOR_BGR2HSV)

	# Create mask for left line
	mask_left = cv2.inRange(frame_hsv,
		np.array([LEFT_H_LOWER, LEFT_S_LOWER, LEFT_V_LOWER]),
		np.array([LEFT_H_UPPER, LEFT_S_UPPER, LEFT_V_UPPER]))

	# Create mask for right line
	mask_right = cv2.inRange(frame_hsv,
		np.array([RIGHT_H_LOWER, RIGHT_S_LOWER, RIGHT_V_LOWER]),
		np.array([RIGHT_H_UPPER, RIGHT_S_UPPER, RIGHT_V_UPPER]))

	# Create regions arrays
	left_regions = np.array([[False for x in range(20)] for y in range(20)])
	right_regions = np.array([[False for x in range(20)] for y in range(20)])

	# Search for lines from bottom to top and from right to left
	for y in range(20):
		for x in range(20):
			# Convert to image coordinates
			x_start = (19 - x) * width // 20
			y_start = (19 - y) * height // 20
			x_end = x_start + width // 20 - 1
			y_end = y_start + height // 20 - 1

			# Count pixels for both colors
			left_pixels = cv2.countNonZero(mask_left[y_start:y_end, x_start:x_end])
			right_pixels = cv2.countNonZero(mask_right[y_start:y_end, x_start:x_end])

			# Fill regions arrays
			if left_pixels > 20:
				left_regions[19 - y, 19 - x] = True
			if right_pixels > 20:
				right_regions[19 - y, 19 - x] = True
			
			# Draw regions
			if left_pixels > 20 and right_pixels > 20:
				cv2.rectangle(frame_debug, (x_start, y_start), (x_end, y_end), (100, 100, 100), 1)
			elif left_pixels > 20:
				cv2.rectangle(frame_debug, (x_start, y_start), (x_end, y_end), (100, 100, 0), 1)
			elif right_pixels > 20:
				cv2.rectangle(frame_debug, (x_start, y_start), (x_end, y_end), (0, 0, 100), 1)
	
	# Find lines
	x_start_left, y_start_left, x_stop_left, y_stop_left, length_left = find_line(left_regions)
	x_start_right, y_start_right, x_stop_right, y_stop_right, length_right = find_line(right_regions)

	# Lines detected flags
	line_left = False
	line_right = False

	# Check both lines
	if cv2.countNonZero(mask_left) < 20000 and cv2.countNonZero(mask_right) < 20000:
		if x_start_left >= 0 and y_start_left >= 0 and x_stop_left >= 0 and y_stop_left >= 0 and length_left > 2:
			line_left = True
		if x_start_right >= 0 and y_start_right >= 0 and x_stop_right >= 0 and y_stop_right >= 0 and length_right > 2:
			line_right = True

	# Manually disable lines
	if line_use == LINES_NOTHING:
		line_left = False
		line_right = False
	elif line_use == LINES_LEFT_ONLY:
		line_right = False
	elif line_use == LINES_RIGHT_ONLY:
		line_left = False

	# Draw left line
	if line_left:
		cv2.line(frame_debug,
			(x_start_left * width // 20, y_start_left * height // 20),
			(x_stop_left * width // 20,  y_stop_left * height // 20),
			(200, 200, 0), 3)

	# Draw right line
	if line_right:
		cv2.line(frame_debug,
			(x_start_right * width // 20, y_start_right * height // 20),
			(x_stop_right * width // 20,  y_stop_right * height // 20),
			(0, 0, 200), 3)

	# Warning messages
	if line_left and not line_right:
		cv2.putText(frame_debug, 'No right line', (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 100, 255), 1, cv2.LINE_AA)
	elif not line_left and line_right:
		cv2.putText(frame_debug, 'No left line', (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 100, 255), 1, cv2.LINE_AA)
	elif not line_left and not line_right:
		cv2.putText(frame_debug, 'No lines', (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)


	# Stop robot if no lines detected
	if not line_left and not line_right:
		# Publish is_lines_detected flag
		is_lines_detected.publish(False)

	# Continue processing
	else:
		# Publish is_lines_detected flag
		is_lines_detected.publish(True)

		# Calculate error between X and end of the lines
		setpoint_x = 10
		if line_left and line_right:
			error_x = ((setpoint_x - x_stop_left) + (setpoint_x - x_stop_right)) // 2
		elif line_left:
			error_x = setpoint_x - x_stop_left - 7
		else:
			error_x = setpoint_x - x_stop_right + 3
		
		# Calculate proportional X correction
		center_p_output = error_x * P_TERM_CENTER_X

		# Calculate inclines error between OY and lines
		if line_left:
			incline_error_left = math.atan2(-1, 0) - math.atan2(
				y_stop_left - y_start_left, x_stop_left - x_start_left)
		else:
			incline_error_left = 0

		if line_right:
			incline_error_right = math.atan2(-1, 0) - math.atan2(
				y_stop_right - y_start_right, x_stop_right - x_start_right)
		else:
			incline_error_right = 0


		# Calculate incline error (for debug)
		if line_left and line_right:
			incline_error = (incline_error_left + incline_error_right) / 2
		elif line_left:
			incline_error = incline_error_left
		else:
			incline_error = incline_error_right


		# Calculate proportional output of the incline controller (left line higher priority)
		incline_p_output_left = incline_error_left * P_TERM_INCLINE_LEFT
		incline_p_output_right = incline_error_right * P_TERM_INCLINE_RIGHT

		# Calculate error and correction between Y of the line start and bottom of the frame
		if line_left:
			y_start_error_left = 20 - y_start_left
			center_p_output_left = y_start_error_left * P_TERM_BOTTOM_Y_LEFT
		else:
			y_start_error_left = 0
			center_p_output_left = 0
		if line_right:
			y_start_error_right = y_start_right - 20
			center_p_output_right = y_start_error_right * P_TERM_BOTTOM_Y_RIGHT
		else:
			y_start_error_right = 0
			center_p_output_right = 0

		# Print errors
		cv2.putText(frame_debug, 'X error: ' +
			str(error_x), (0, 40), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1, cv2.LINE_AA)
		cv2.putText(frame_debug, 'Y error (left, right): ' + str(int(y_start_error_left)) 
			+ ', ' + str(int(y_start_error_right)), 
		 (0, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1, cv2.LINE_AA)
		cv2.putText(frame_debug, 'Angle error (deg): ' + str(int(math.degrees(incline_error))), 
		 (0, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1, cv2.LINE_AA)

		# Calculate total angular output
		p_output = center_p_output + incline_p_output_left + incline_p_output_right + center_p_output_left + center_p_output_right

		# Trim output to -0.8 - 0.8
		if p_output > 0.8:
			p_output = 0.8
		elif p_output < -0.8:
			p_output = -0.8	

		# Calculate linear velocity
		linear_velocity = (abs(math.degrees(incline_error)) * -0.14 / 90 + 0.14)
		if linear_velocity > 0.14:
			linear_velocity = 0.14
		elif linear_velocity < 0.05:
			linear_velocity = 0.05

		# Send velocities to the mission_handler
		angular_velocity_publisher.publish(p_output)
		linear_velocity_publisher.publish(linear_velocity)

	# Send debug image
	image_publisher.publish(cvBridge.cv2_to_imgmsg(frame_debug, 'bgr8'))


def find_line(regions):
	# Initialize variables
	index_x = -1
	index_y = -1
	x_start = -1
	y_start = -1
	x_stop = -1
	y_stop = -1
	direction_last = 0

	# Search for begginng of the line from bottom to top and from right to left
	for y in range(20):
		for x in range(20):
			if regions[19 - y, 19 - x]:
				index_x = 19 - x
				index_y = 19 - y
				break
		if index_x >= 0 and index_y >= 0:
			break

	# Store start position
	x_start = index_x
	y_start = index_y

	# Search connected regions
	if index_x >= 0 and index_y >= 0:
		while True:
			# Forward
			if index_y > 0 and regions[index_y - 1, index_x]:
				index_y = index_y - 1
				direction_last = 0

			# Forward left
			elif index_y > 0 and index_x > 0 and regions[index_y - 1, index_x - 1]:
				index_y = index_y - 1
				index_x = index_x - 1
				direction_last = 0

			# Forward right
			elif index_y > 0 and index_x < 19 and regions[index_y - 1, index_x + 1]:
				index_y = index_y - 1
				index_x = index_x + 1
				direction_last = 0

			# Left
			elif index_x > 0 and direction_last <= 0 and regions[index_y, index_x - 1]:
				if direction_last == 0:
					direction_last = -1
				index_x = index_x - 1

			# Right
			elif index_x < 19 and direction_last >= 0 and regions[index_y, index_x + 1]:
				if direction_last == 0:
					direction_last = 1
				index_x = index_x + 1

			# Line finished
			else:
				x_stop = index_x
				y_stop = index_y
				break

	# Calculate line length
	length = math.sqrt((x_stop - x_start) * (x_stop - x_start) 
		+ (y_stop - y_start) * (y_stop - y_start))

	# Return start, end and lendth of the line
	return (x_start, y_start, x_stop, y_stop, length)


def line_use_callback(data):
	global line_use
	line_use = data.data


# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	rospy.init_node('line_detector_mission_gui')

	# Create subscriber for camera image
	rospy.Subscriber('/camera/image', Image, image_callback, queue_size = 1)

	# Create subscriber for line usage
	rospy.Subscriber('line_use', Int16, line_use_callback, queue_size = 1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

	# Exit message
	print('line_detector_mission_gui finished')
