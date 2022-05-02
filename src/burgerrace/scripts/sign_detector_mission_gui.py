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
from os import listdir
from os.path import isfile, join

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, String

# --- DATASET ---
# Folder with images
IMAGES_FOLDER = '/home/sim/Desktop/ros_workspace/src/burgerrace/scripts/images/'
IMAGE_EXTENSION = 'png'

# Publisher for debug image
image_publisher = rospy.Publisher('sign_image', Image, queue_size=1)

# Publisher for list of strings
detected_signs_publisher = rospy.Publisher('detected_signs', String, queue_size=1)

# OpenCV bridge for camera image
cvBridge = CvBridge()

# Dataset arrays
rgb_colors_lower = []
rgb_colors_upper = []
images_names = []
images_bgr = []

# Detected signs
detected_signs = []


def prepare_dataset():
	"""
	Loads dataset to the color keys, filenames and bgr images
	"""
	global rgb_colors_lower, rgb_colors_upper
	global images_names, images_bgr

	# List all files in directory
	onlyfiles = [f for f in listdir(IMAGES_FOLDER) if isfile(join(IMAGES_FOLDER, f))]
	for file in onlyfiles:
		# Split name and extension
		splitted_name = file.split('.')

		# Name is the first part
		name = splitted_name[0]

		# Extension is the last part
		extension = splitted_name[len(splitted_name) - 1]

		# Accept only selected extensions
		if extension == IMAGE_EXTENSION:
			# Read image from file
			image = cv2.imread(IMAGES_FOLDER + '/' + file)

			# Resize to 64x64
			#image = cv2.resize(image, (64, 64))

			# Calculate most frequent non-white color
			top_color = unique_count_app(image)

			# Print image info
			print('-------------')
			print('Found image: ' + name + ' in ' + IMAGES_FOLDER)
			print('Top color in this image (BGR): ' + str(top_color))

			# Calculate edges for each picture
			blue_lower = min(255, max(0, top_color[0] - 12))
			blue_upper = min(255, max(0, top_color[0] + 12))
			green_lower = min(255, max(0, top_color[1] - 12))
			green_upper = min(255, max(0, top_color[1] + 12))
			red_lower = min(255, max(0, top_color[2] - 12))
			red_upper = min(255, max(0, top_color[2] + 12))
			
			# Append color keys to the lists
			rgb_colors_lower.append([blue_lower, green_lower, red_lower])
			rgb_colors_upper.append([blue_upper, green_upper, red_upper])

			# Append image and filename to the lists
			images_bgr.append(image)
			images_names.append(name)
			
	# Convert to numpy types
	rgb_colors_lower = np.array(rgb_colors_lower)
	rgb_colors_upper = np.array(rgb_colors_upper)
	images_bgr = np.array(images_bgr)
	images_names = np.array(images_names)


def image_callback(data):
	"""
	Main OpenCV image callback
	"""
	global rgb_colors_lower, rgb_colors_upper
	global images_names, images_bgr
	global detected_signs

	# Get source image
	frame_source = cvBridge.imgmsg_to_cv2(data, 'bgr8')

	# Get size of image
	height, width, _ = frame_source.shape

	# Copy warped frame
	frame_debug = frame_source.copy()

	# Lists of found images
	found_image_indexes = []
	found_image_rects = []
	found_image_dencity = []
	found_image_course = []
	found_image_distance = []

	# List through all images
	for i in range(len(images_bgr)):

		# Calculate mask and contours
		mask = cv2.inRange(frame_source, rgb_colors_lower[i], rgb_colors_upper[i])
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Filter contours by area and dencity (dencity > 25%, area > 200, area < 6000)
		contours_filtered = []
		for k in range(len(contours)):
			x, y, w, h = cv2.boundingRect(contours[k])
			if cv2.countNonZero(mask[y: y + h, x: x + w]) > (w * h) // 4 and w * h < 6000 and w * h > 50:
				contours_filtered.append(contours[k])
		contours_filtered = np.array(contours_filtered)

		# Sort contours by area
		contours_filtered = sorted(contours_filtered, key=lambda x: cv2.contourArea(x))

		if len(contours_filtered) > 1:
			contours_merged = []

			# Middle area contour
			middle_area_index = len(contours_filtered) // 2
			middle_x, middle_y, middle_w, middle_h = cv2.boundingRect(contours_filtered[middle_area_index])

			# Find and merge two similar contours
			merged = False
			for k in range(len(contours_filtered)):
				# Get rectangle for each contour
				x, y, w, h = cv2.boundingRect(contours[k])

				# If it is not middle contour
				if k != middle_area_index:
						# Check if it can be merged
						if abs(w - middle_w) < 20 and abs(h - middle_h) < 20 and abs(x - middle_x) < 200 and abs(y - middle_y) < 200:
							# Merge contours
							x_min = min(middle_x, x)
							y_min = min(middle_y, y)
							x_max = max(middle_x, x)
							y_max = max(middle_y, y)
							w_max = max(middle_w, w)
							h_max = max(middle_h, h)
							contours_merged.append(np.array([[[x_min, y_min]],
														[[x_max + w_max, y_min]],
														[[x_max + w_max, y_max + h_max]],
														[[x_min, y_max + h_max]]]))

							# Set merget flag and exit from loop
							merged = True
							break

			# If two contours was merged, append other ones
			if merged:
				for j in range(len(contours_filtered)):
					if j != k and j != middle_area_index:
						contours_merged.append(contours[j])

				# Convert to numpy array
				contours_filtered = np.array(contours_merged)

		# Check number of contours
		if len(contours_filtered) > 0:

			# Draw all filtered contours
			#cv2.drawContours(frame_debug, contours_filtered, -1, (0, 255, 0), 1)

			# Get contour with maximum area
			max_contour = contours_filtered[0]

			# Calculate bounding rectangle
			x, y, w, h = cv2.boundingRect(max_contour)

			# Get size of reference image
			ref_height, ref_width, _ = images_bgr[i].shape

			# Check aspect ratio
			if abs(w / h - ref_width / ref_height) < 0.4:
				# Create transformation matrix
				transform_src = np.array([
				[x, y],
				[x + w, y],
				[x + w, y + h],
				[x, y + h]], dtype = 'float32')
				transform_dst = np.array([
					[0, 0],
					[ref_width, 0],
					[ref_width, ref_height],
					[0, ref_height]], dtype = 'float32')
				perspective_matrix = cv2.getPerspectiveTransform(transform_src, transform_dst)

				# Transform sign to 64x64
				sign_warped = cv2.warpPerspective(frame_source, perspective_matrix, (ref_width, ref_height))

				# Find difference between reference image and found sign
				difference = cv2.absdiff(sign_warped, images_bgr[i])

				# Invert frame to calculate equal pixels
				same_pixels = cv2.bitwise_not(difference)

				# Find white pixels
				same_pixels = cv2.inRange(same_pixels, np.array([200, 200, 200]), np.array([255, 255, 255]))

				# Calculate number of equeal pixels
				same_pixels_num = cv2.countNonZero(same_pixels)

				# Calculate maximum possible number of white pixels
				total_pixels_reference = ref_height * ref_width

				# Threshold number of pixel to 60%
				if same_pixels_num / total_pixels_reference > 0.60:
					# Append found image data to lists
					found_image_indexes.append(i)
					found_image_rects.append([x, y, w, h])
					found_image_dencity.append(min(1., max(0., same_pixels_num / total_pixels_reference)))
					found_image_course.append(int((x + w // 2) * 100 / 320 - 50))
					found_image_distance.append(80 - ((w + h) // 2))

	# List of all detected signs
	found_image_names = []

	# Draw detected rectangles and debug string
	for i in range(len(found_image_indexes)):
		# Extract parameters
		index = found_image_indexes[i]
		[x, y, w, h] = found_image_rects[i]
		dencity = found_image_dencity[i]
		course = found_image_course[i]
		distance = found_image_distance[i]

		# Calculate color of the border, text and other elements
		blue = int(rgb_colors_upper[index][0])
		green = int(rgb_colors_upper[index][1])
		red = int(rgb_colors_upper[index][2])

		# Fill lists of names for mission handler
		# Parking sign (minimum distance 40)
		if images_names[index] == 'PARKING':
			if distance < 40:
				found_image_names.append(images_names[index])
		# Tonnel and stop sign (minimum distance 50)
		elif images_names[index] == 'STOP' or images_names[index] == 'TONNEL':
			if distance < 50:
				found_image_names.append(images_names[index])
		# BIBOT sign (minimum distance 70)
		elif images_names[index] == 'BIBOT':
			if course > -45:
				found_image_names.append(images_names[index])
		# Other signs (any distance)
		else:
			found_image_names.append(images_names[index])

		# Draw course
		cv2.line(frame_debug, (160, 240), (int(x + w // 2), int(y + h // 2)), (blue, green, red), 1)

		# Draw rectangle
		cv2.rectangle(frame_debug, (x,y), (x + w, y + h), (255 - blue, 255 - green, 255 - red), 2)

		# Draw debug strings
		cv2.putText(frame_debug, images_names[index] + ' ' + str(int(dencity * 100)) + '%', (x, y + h + 15), cv2.FONT_HERSHEY_PLAIN, 1, (255 - blue, 255 - green, 255 -red), 1, cv2.LINE_AA)
		cv2.putText(frame_debug, str(course) + 'deg, Dst: ' + str(distance), (x, y + h + 30), cv2.FONT_HERSHEY_PLAIN, 1, (255 - blue, 255 - green, 255 -red), 1, cv2.LINE_AA)
		
	# Draw text of detected sigs in right order
	for i in range(len(detected_signs)):
		name = detected_signs[i]
		cv2.putText(frame_debug, name, (10, i * 20 + 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 0), 1, cv2.LINE_AA)

	# Send list of strings
	detected_signs_publisher.publish(','.join(found_image_names))

	# Send debug image
	image_publisher.publish(cvBridge.cv2_to_imgmsg(frame_debug, 'bgr8'))


# Finds most frequent color in the image
def unique_count_app(image):

	# Find colors and frequencies
    colors, count = np.unique(image.reshape(-1, image.shape[-1]), axis=0, return_counts=True)

    # Merge and sorts lists by number of pixels
    colors_and_count = []
    for i in range(len(colors)):
    	colors_and_count.append([colors[i], count[i]])
    colors_and_count.sort(key=lambda x:x[1], reverse=True)

    # Split lists back
    colors = []
    count = []
    for color_and_count in colors_and_count:
    	colors.append(color_and_count[0])
    	count.append(color_and_count[1])

    # Find first non-white color
    max_index = 0
    while colors[max_index][0] > 200 and colors[max_index][1] > 200 and colors[max_index][2] > 200 and max_index < len(colors) - 1:
    	max_index += 1

    # Return color
    return colors[max_index]


# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	rospy.init_node('sign_detector_mission_gui')

	prepare_dataset()

	# Create subscriber for camera image
	rospy.Subscriber('/camera_2/image_2', Image, image_callback, queue_size = 1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break
			cv2.destroyAllWindows()

	# Exit message
	print('sign_detector_mission_gui finished')
