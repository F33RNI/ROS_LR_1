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


# --- SETUP ---
# Scale
SCALE = 50

# Radius of the points
POINT_RADIUS = 1

# Obstacle distances
OBSTACLE_DIST_FRONT = 0.3
OBSTACLE_DIST_SIDE = 0.5


# Publisher for debug image
image_publisher = rospy.Publisher('lidar_image', Image, queue_size=1)

# Publishers for obstacles
front_obstacle = rospy.Publisher('front_obstacle', Bool, queue_size=1)
left_obstacle = rospy.Publisher('left_obstacle', Bool, queue_size=1)
right_obstacle = rospy.Publisher('right_obstacle', Bool, queue_size=1)

# OpenCV bridge
cvBridge = CvBridge()

def rotate2d(pos, rad):
	"""
	Rotates point on angle
	:param pos: point
	:param rad: angle in radians
	:return:
	"""
	x, y = pos
	s, c = math.sin(rad), math.cos(rad)
	return x * c - y * s, y * c + x * s


def lidar_callback(data):
	"""
	Draws debug lidar data
	"""
	frame_debug = 255 * np.ones((400, 400, 3), np.uint8)

	# Draw robot (center) circle
	cv2.circle(frame_debug, (200, 200), 5, (0, 0, 0), 1)

	# Draw robot direction (from bottom to the top)
	cv2.line(frame_debug, (200, 200), (200, 180), (0, 0, 0), 1)

	# Get array of distances (size of array is 360 elements)
	ranges = data.ranges


	# Draw referce lines
	# Front
	draw_obstacle_line(frame_debug, 340, 20, OBSTACLE_DIST_FRONT, (0, 0, 150))
	# Left
	draw_obstacle_line(frame_debug, 340, 290, OBSTACLE_DIST_SIDE, (0, 150, 0))
	# Right
	draw_obstacle_line(frame_debug, 70, 20, OBSTACLE_DIST_SIDE, (150, 0, 0))

	# Calcuulate front obstacle
	front_distance = calculate_min_distance(ranges, 340, 360, 0, 20)
	if front_distance != math.inf:
		draw_obstacle_line(frame_debug, 340, 20, front_distance, (0, 0, 255))

	if front_distance <= OBSTACLE_DIST_FRONT:
		front_obstacle.publish(True)
	else:
		front_obstacle.publish(False)

	# Calcuulate left obstacle
	left_distance = calculate_min_distance(ranges, 290, 340)
	if left_distance != math.inf:
		draw_obstacle_line(frame_debug, 340, 290, left_distance, (0, 255, 0))

	if left_distance <= OBSTACLE_DIST_SIDE:
		left_obstacle.publish(True)
	else:
		left_obstacle.publish(False)

	# Calcuulate right obstacle
	right_distance = calculate_min_distance(ranges, 20, 70)
	if right_distance != math.inf:
		draw_obstacle_line(frame_debug, 70, 20, right_distance, (255, 0, 0))

	if right_distance <= OBSTACLE_DIST_SIDE:
		right_obstacle.publish(True)
	else:
		right_obstacle.publish(False)

	# Draw all elements
	for i in range(len(ranges)):
		# Mirror distance
		distance = ranges[359 - i]

		# Check for infinity
		if distance != math.inf:

			# Scale distance
			distance *= SCALE

			# Calculated rotated point relative to the 0, 0
			rotated_x, rotated_y = rotate2d((0, -distance), math.radians(i))

			# Add center of the image
			rotated_x += 200
			rotated_y += 200

			# Trim maximum value to 180 for correct color conversion
			if distance > 180:
				distance = 180

			# Create HSV color depending on distance
			color = np.uint8([[[int(distance), 255, 255]]])

			# Convert color to the BGR space
			colorBGR = cv2.cvtColor(color, cv2.COLOR_HSV2BGR)

			# Draw each pixel as filled circle
			cv2.circle(frame_debug, (int(rotated_x), int(rotated_y)), POINT_RADIUS, (int(colorBGR[0][0][0]), int(colorBGR[0][0][1]), int(colorBGR[0][0][2])), -1)

	# Send debug image
	image_publisher.publish(cvBridge.cv2_to_imgmsg(frame_debug, 'bgr8'))



def calculate_min_distance(ranges, angle_start_first_deg, angle_stop_first_deg, angle_start_second_deg = 0, angle_stop_second_deg = 0):
	"""
	Calculates minimum distance to the obstacle in defined angles
	"""
	min_value = math.inf

	# First range
	for i in range(angle_start_first_deg, angle_stop_first_deg):
		distance = ranges[i]
		if distance != math.inf and distance < min_value:
			min_value = distance

	# Second range
	if angle_start_second_deg != angle_stop_second_deg:
		for i in range(angle_start_second_deg, angle_stop_second_deg):
			distance = ranges[i]
			if distance != math.inf and distance < min_value:
				min_value = distance

	return min_value

def draw_obstacle_line(frame, line_start_deg, line_stop_deg, distance, color):
	"""
	Draws colored line between to angles with distance between center
	"""
	distance *= SCALE
	left_x, left_y = rotate2d((0, -distance), math.radians(line_start_deg))
	right_x, right_y = rotate2d((0, -distance), math.radians(line_stop_deg))
	cv2.line(frame, (int(left_x + 200), int(left_y + 200)), (int(right_x + 200), int(right_y + 200)), color, 1)



# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	rospy.init_node('lidar_reader')

	# Create subscriber for lidar
	rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

	# Exit message
	print('lidar_reader finished')
