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
import sys
from os import listdir
from os.path import isfile, join
import signal
import threading

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool, String, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from PyQt5 import uic, QtGui, Qt, QtCore
from PyQt5.QtGui import QPen, QColor
from PyQt5.QtWidgets import QApplication, QFileDialog, QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtGui import QPixmap
from python_qt_binding import loadUi
import qimage2ndarray


SPEED_INCREMENT = 0.05

IMAGE_TYPE_FRONT_RAW = 0
IMAGE_TYPE_FRONT_SIGN = 1
IMAGE_TYPE_TOP_RAW = 2
IMAGE_TYPE_TOP_LINE = 3


class Window(QMainWindow):
	update_logs = QtCore.Signal(str)
	update_front_image = QtCore.Signal(QPixmap)
	update_top_image = QtCore.Signal(QPixmap)
	update_lidar_image = QtCore.Signal(QPixmap)
	update_odom_x = QtCore.Signal(str)
	update_odom_y = QtCore.Signal(str)
	update_odom_yaw = QtCore.Signal(str)

	def __init__(self):
		super(Window, self).__init__()

		# OpenCV bridge for camera image
		self.cvBridge = CvBridge()

		# Twist class for manual control
		self.twist = Twist()

		# Is running flag for manual control thread
		self.manual_control_thread_running = False

		# Variables for storing the absolute position of the robot by odometry
		self.x = 0
		self.y = 0
		self.yaw = 0

		# Variables for storing the relative position of the robot by odometry
		self.odom_x_tare = 0
		self.odom_y_tare = 0
		self.odom_yaw_tare = 0

		# Flag to start odometry initialization 
		self.is_odom_initialized = False

		# Find GUI directory
		self_dir = os.path.dirname(os.path.realpath(__file__))
		print('GUI directiry: ', self_dir)

		# Find and load GUI file
		ui_file = os.path.join(self_dir, 'gui.ui')
		loadUi(ui_file, self)

		# ROS initialization 
		rospy.init_node('gui_controller')

		# Front raw camera image
		rospy.Subscriber('/camera_2/image_2', Image, self.camera_image_callback, IMAGE_TYPE_FRONT_RAW, queue_size=1)

		# Top raw camera image
		rospy.Subscriber('/camera/image', Image, self.camera_image_callback, IMAGE_TYPE_TOP_RAW, queue_size=1)

		# Sign detection image
		rospy.Subscriber('sign_image', Image, self.camera_image_callback, IMAGE_TYPE_FRONT_SIGN, queue_size=1)

		# Line detection image
		rospy.Subscriber('image_line', Image, self.camera_image_callback, IMAGE_TYPE_TOP_LINE, queue_size=1)

		# Lidar image
		rospy.Subscriber('lidar_image', Image, self.lidar_image_callback, queue_size=1)

		# Debug log
		rospy.Subscriber('mission_logs', String, self.mission_logs_callback, queue_size=8)

		# Create subscriber for odometry
		rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

		# Create publishers
		self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.manual_control = rospy.Publisher('manual_control', Bool, queue_size=1)

		# Disable autopilot
		self.manual_control.publish(True)

		# Connect GUI controls
		self.btn_forward.clicked.connect(self.manual_forward)
		self.btn_backward.clicked.connect(self.manual_backward)
		self.btn_stop.clicked.connect(self.manual_stop)
		self.btn_left.clicked.connect(self.manual_left)
		self.btn_right.clicked.connect(self.manual_right)
		self.radio_manual.clicked.connect(self.manual_begin)
		self.radio_apilot.clicked.connect(self.manual_end)
		self.btn_odom_clear.clicked.connect(self.odom_clear)

		# Connect signals
		self.update_logs.connect(self.debug_log.appendPlainText)
		self.update_front_image.connect(self.front_camera.setPixmap)
		self.update_top_image.connect(self.top_camera.setPixmap)
		self.update_lidar_image.connect(self.lidar_image.setPixmap)
		self.update_odom_x.connect(self.line_odom_x.setText)
		self.update_odom_y.connect(self.line_odom_y.setText)
		self.update_odom_yaw.connect(self.line_odom_yaw.setText)

		# Switch to manual control
		self.radio_manual.setChecked(True)
		self.manual_begin()


	def manual_forward(self):
		"""
		Increases linear X speed of the robot
		"""
		self.twist.linear.x += SPEED_INCREMENT

	def manual_backward(self):
		"""
		Decreases linear X speed of the robot
		"""
		self.twist.linear.x -= SPEED_INCREMENT

	def manual_stop(self):
		"""
		Resets linear and angular speeds
		"""
		self.twist.linear.x = 0
		self.twist.angular.z = 0

	def manual_left(self):
		"""
		Increases angular Z speed of the robot
		"""
		self.twist.angular.z += SPEED_INCREMENT

	def manual_right(self):
		"""
		Decreases angular Z speed of the robot
		"""
		self.twist.angular.z -= SPEED_INCREMENT

	def manual_begin(self):
		"""
		Disables automatic movement and starts manual control thread
		"""
		if not self.manual_control_thread_running:
			self.manual_control_thread_running = True
			self.manual_control.publish(True)
			self.twist.linear.x = 0
			self.twist.angular.z = 0
			threading.Thread(target=self.manual_control_loop).start()

	def manual_end(self):
		"""
		Stops the manual control thread and allows automatic control
		"""
		if self.manual_control_thread_running:
			self.manual_control_thread_running = False
			self.manual_control.publish(False)

	def manual_control_loop(self):
		"""
		Manual control thread loop
		"""
		while self.manual_control_thread_running:
			self.cmd_vel.publish(self.twist)

	def camera_image_callback(self, data, type):
		"""
		Displays various images from subscribers on form elements
		"""
		frame = self.cvBridge.imgmsg_to_cv2(data, 'bgr8')
		pixmap = QPixmap.fromImage(qimage2ndarray.array2qimage(
			cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (320, 240), interpolation=cv2.INTER_NEAREST)))

		# Raw front camera image
		if type == IMAGE_TYPE_FRONT_RAW:
			if self.radio_front_raw.isChecked():
				self.update_front_image.emit(pixmap)

		# Raw top camera image
		elif type == IMAGE_TYPE_TOP_RAW:
			if self.radio_top_raw.isChecked():
				self.update_top_image.emit(pixmap)

		# Sign detection with front camera
		elif type == IMAGE_TYPE_FRONT_SIGN:
			if self.radio_front_sign.isChecked():
				self.update_front_image.emit(pixmap)

		# Line detection with top camera
		elif type == IMAGE_TYPE_TOP_LINE:
			if self.radio_top_line.isChecked():
				self.update_top_image.emit(pixmap)


	def lidar_image_callback(self, data):
		"""
		Draws lidar image (from subscriber) on form element
		"""
		frame = self.cvBridge.imgmsg_to_cv2(data, 'bgr8')

		self.update_lidar_image.emit(QPixmap.fromImage(qimage2ndarray.array2qimage(
                cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (200, 200), interpolation=cv2.INTER_NEAREST))))


	def mission_logs_callback(self, data):
		"""
		Updates debug_log element
		"""
		self.update_logs.emit(str(data.data))

	def odom_callback(self, data):
		"""
		Displays the relative values of the robot's odometers
		"""
		pose = data.pose.pose

		# Caculate current (absolute) x, y positions and yaw angle of the robot
		self.x = pose.position.x
		self.y = pose.position.y
		_, _, self.yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

		# Calculate releative x, y positions and yaw angle
		x_rel = self.x - float(self.spinbox_odom_start_x.value()) - self.odom_x_tare
		y_rel = self.y - float(self.spinbox_odom_start_y.value()) - self.odom_y_tare
		yaw_rel = self.yaw - math.radians(float(self.spinbox_odom_start_yaw.value())) - self.odom_yaw_tare

		# Displaying relative values
		self.update_odom_x.emit(str(round(x_rel, 3)))
		self.update_odom_y.emit(str(round(y_rel, 3)))
		self.update_odom_yaw.emit(str(round(math.degrees(yaw_rel), 3)))


	def odom_clear(self):
		"""
		Resets releative position
		"""
		# Calculate releative x, y positions and yaw angle
		x_rel = self.x - float(self.spinbox_odom_start_x.value())
		y_rel = self.y - float(self.spinbox_odom_start_y.value())
		yaw_rel = self.yaw - math.radians(float(self.spinbox_odom_start_yaw.value()))

		self.odom_x_tare = x_rel
		self.odom_y_tare = y_rel
		self.odom_yaw_tare = yaw_rel

		# Displaying relative values
		self.update_odom_x.emit('0')
		self.update_odom_y.emit('0')
		self.update_odom_yaw.emit('0')



# Main entry point
if __name__ == '__main__':
	app = QApplication(sys.argv)
	app.setStyle('fusion')
	win = Window()
	win.show()
	sys.exit(app.exec_())
