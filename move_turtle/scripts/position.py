#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from move_turtle.srv import *

position=Pose

def sendPosition(requested):
	print(requested)
	if requested.c==1:
		print(position)
		x=position.x
		y=position.y
		theta=position.theta
		return newPositionResponse(position.x, position.y, position.theta, position.linear_velocity, position.angular_velocity)

def collect(data):
	global position
	position = data

def main():
	global position
	rospy.init_node('position', anonymous=True)	
	rospy.Subscriber("/turtle1/pose", Pose, collect)
	s = rospy.Service('positionService', newPosition, sendPosition)
	rospy.spin()

if __name__ == "__main__":
	main()	