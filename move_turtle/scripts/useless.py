if ((endPoint[1]-startPoint[1])>=0 and startPoint[2]>=0):
			velocity.angular.z=startPoint[2]-theta
			if startPoint[2]>=0:
				angle=startPoint[2]
			else:
				angle=startPoint[2]+8*(math.atan(1))

		else:
			velocity.angular.z=startPoint[2]-theta+4*(math.atan(1))
			direction=-1.0*(direction)
			if startPoint[2]>=0:
				angle=startPoint[2]
			else:
				angle=startPoint[2]+8*(math.atan(1))
((abs(position.x-endPoint[1]))>0.01 or (abs(position.y-endPoint[0]))>0.01) and 
abs(angle-position.theta)>0.0001 and 

rospy.Subscriber("/turtle1/pose", Pose, stop)
rospy.Subscriber("/turtle1/pose", Pose, stop)
rospy.Subscriber("/turtle1/pose", Pose, stop)

			rospy.Subscriber("/turtle1/pose", Pose, stop)

def stop(data):
	global position
	position=data

def turtleMove(path, numberNodes):
	global position

	rospy.init_node('moveTurtle', anonymous=True)

	velocityPublisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	startPoint=path.popleft()

	rospy.wait_for_service('/turtle1/teleport_absolute')
	teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teleport(startPoint[1],startPoint[0],0)
	velocity= Twist()
	theta=0
	rospy.Subscriber("/turtle1/pose", Pose, stop)
	#print(position)

	for i in range(0,numberNodes-1):

		angle=None
		endPoint=path.popleft()
		velocity.linear.x=0
		velocity.linear.z=0
		velocity.angular.x=0
		velocity.angular.y=0
		if (endPoint[1]-startPoint[1])>=0 :

			if startPoint[2]>=0:
				angle=startPoint[2]
			else:
				angle=startPoint[2]+8*(math.atan(1))
		else:
			angle=startPoint[2]+4*(math.atan(1))	

		rotated=0
		print(angle-theta)
		currentTime=time.time()
		while abs(abs(rotated)-abs(angle-theta))>0.0001 :
			velocity.angular.z=(angle-theta-rotated)
			velocityPublisher.publish(velocity)
			rotated=rotated+velocity.angular.z*(time.time()-currentTime)
			currentTime=time.time()
		print(rotated)	

		velocity.angular.z=0	
		velocityPublisher.publish(velocity)	
		velocity.linear.x=startPoint[3]/20
		
		distance=math.sqrt((endPoint[1]-startPoint[1])*(endPoint[1]-startPoint[1])+(endPoint[0]-startPoint[0])*(endPoint[0]-startPoint[0]))	
		travelled=0
		currentTime=time.time()
		while (abs(distance- travelled))>0.01:

			velocityPublisher.publish(velocity)
			#print([endPoint[1],endPoint[0]],[position.x,position.y])
			travelled=velocity.linear.x*(time.time()-currentTime)

		velocity.linear.x=0	
		velocityPublisher.publish(velocity)
		startPoint=endPoint	
		theta=angle
	print(time.time()-startTime)			