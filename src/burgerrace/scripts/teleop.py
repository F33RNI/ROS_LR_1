#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
import sys, select, os
from std_msgs.msg import Bool
if os.name == 'nt':
	import msvcrt
else:
	import tty, termios

BURGER_MAX_LIN_VEL = 0.5
BURGER_MAX_ANG_VEL = 5

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.03
ANG_VEL_STEP_SIZE = 5

msg = """
Moving around:
     w
a    s    d

space key: force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

manual_control_publisher = rospy.Publisher('manual_control', Bool, queue_size=1)

def getKey():
	if os.name == 'nt':
		if sys.version_info[0] >= 3:
			return msvcrt.getch().decode()
		else:
			return msvcrt.getch()

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output


def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input

	return input


def checkLinearLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

	return vel


def checkAngularLimitVelocity(vel):
	if turtlebot3_model == "burger":
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
	elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
		vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
	else:
		vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

	return vel


if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('turtlebot3_teleop')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	turtlebot3_model = rospy.get_param("model", "burger")

	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0



	try:
		#print(msg)
		rospy.sleep(1)
		while(1):

			#
		
			target_angular_vel = 0.0
		
			key = getKey()
			
			if key == 'w' :
				if target_linear_vel >= 0.0:
					target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
				else:
					target_linear_vel = 0.0
				manual_control_publisher.publish(True)
			
			elif key == 's':
				if target_linear_vel <= 0.0:
					target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
				else:
					target_linear_vel = 0.0
				manual_control_publisher.publish(True)
			
			elif key == 'a':
				if target_angular_vel >= 0.0:
					target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
				else:
					target_angular_vel = 0.0
				manual_control_publisher.publish(True)
			
			elif key == 'd':
				if target_angular_vel <= 0.0:
					target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
				else:
					target_angular_vel = 0.0
				manual_control_publisher.publish(True)
			
			elif key == ' ':
				target_linear_vel   = 0.0
				control_linear_vel  = 0.0
				target_angular_vel  = 0.0
				control_angular_vel = 0.0
				manual_control_publisher.publish(False)
			else:
				if (key == '\x03'):
					break

			twist = Twist()

			control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
			twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

			control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			
			pub.publish(twist)

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
