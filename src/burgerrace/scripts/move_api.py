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

import rospy, sys, select, os, tty, termios
from time import sleep
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

# --- SET FORWARD SPEED HERE ---
FORWARD_SPEED = 0.14

# Publisher for velocity
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Movement flag
is_moving = False

# Velocities
linear_velocity = 0.
angular_velocity = 0.

manual_control = False


def linear_velocity_callback(velocity):
	"""
	Reads linear_velocity from Subscriber
	"""
	global linear_velocity
	linear_velocity = velocity.data
	set_velocity()


def angular_velocity_callback(velocity):
	"""
	Reads angular_velocity from Subscriber
	"""
	global angular_velocity
	angular_velocity = velocity.data
	set_velocity()


def set_velocity():
	"""
	Pushes linear.x and angular.z velocity to the Twist() class
	"""
	global is_moving

	twist = Twist()

	# Forward movement
	if is_moving:
		twist.linear.x = linear_velocity
	else:
		twist.linear.x = 0.

	twist.linear.y = 0.
	twist.linear.z = 0.
	twist.angular.x = 0.
	twist.angular.y = 0.

	# Rotation
	twist.angular.z = angular_velocity

	# Publish data
	if not manual_control:
		cmd_vel.publish(twist)


def move_callback(is_move):
	"""
	Starts or stops the robot
	"""
	global is_moving
	is_moving = is_move.data

	# Start or stop the robot
	set_velocity()


def manual_control_callback(data):
	"""
	Blocks publishing speeds if manual_control is True
	"""
	global manual_control
	manual_control = data.data
	

# Main entry point
if __name__ == '__main__':
	# ROS initialization 
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('move_api')

	# Connect subscriber for linear_x variable
	rospy.Subscriber('linear_velocity', Float64, linear_velocity_callback, queue_size = 1)

	# Connect subscriber for angular_z variable
	rospy.Subscriber('angular_velocity', Float64, angular_velocity_callback, queue_size = 1)

	# Connect subscriber for move variable
	rospy.Subscriber('move', Bool, move_callback, queue_size = 1)

	# Connect subscriber for block control
	rospy.Subscriber('manual_control', Bool, manual_control_callback, queue_size=1)

	# ROS loop
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break

	# Exit message
	print('move_api finished')
