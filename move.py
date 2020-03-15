#!/usr/bin/env python
import cv2
import math
import heapq
import random
import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from moveTurtle.srv import TeleportAbsolute
from collections import deque

###create a list of nodes for source and destination
startTime=time.time()
l=5
size=[]
xSource=0
ySource=0
xDestination=0
yDestination=0

#finds source and destination points
def sourceDestinationPoints(img):
	global xSource
	global ySource
	global xDestination
	global yDestination
	global size

	for y in range(size[0]):
		for x in range(size[1]):
			if img.item(y,x,1)>200 and img.item(y,x,2)<100 and img.item(y,x,0)<100:
				xSource=x
				ySource=y
				#print(xSource,ySource)
				break;

		if xSource==x and	ySource==y:
			break;	

	for y in range(size[0]):
		for x in range(size[1]):
			if img.item(y,x,2)>200 and img.item(y,x,1)<100 and img.item(y,x,0)<100:
				xDestination=x
				yDestination=y
				break;

		if xDestination==x and	yDestination==y:
			break;			

#checks node is inside image
def valid(x, y):
	global size

	if (x >=0 and y >= 0) and (x < size[1] and y < size[0]): 
		return 1
	else:
		return 0	

###edit it to be more efficient, use list of nodes
#checks closest vertex to random point
def closestVertex(xRad, yRad, listNode):###lst

	xClosestVertex=None
	yClosestVertex=None
	distanceClosestVertex=100000

	for i in listNode:###lst
		distanceCurrentVertex = math.sqrt( (xRad-i[1])*(xRad-i[1]) + (yRad-i[0])*(yRad-i[0]) )
		if  distanceCurrentVertex< distanceClosestVertex:
			distanceClosestVertex = distanceCurrentVertex
			xClosestVertex = i[1]
			yClosestVertex = i[0]

	return [yClosestVertex, xClosestVertex,distanceClosestVertex]				

#checks if point is on obstacle
def notObstacle(y, x, img):

	if int(img.item(y,x,0))<50 or int(img.item(y,x,1))<50 or int(img.item(y,x,2))<50 :
		return 1
	else:
		#print('found obstacle')
		return 0


#checks lowest cost in vicinity
def lowestCostNode(nodeTree, xProbableNode, yProbableNode, costNode):
	global size

	xvals=[]
	yvals=[]
	nodeDistance=None
	xParentNode=None
	yParentNode=None
	lcstNodeDistance= 10000000000000000000000000000
	probableRemaps=[]

	for i in range(xProbableNode-15, xProbableNode+15):
		if i>=0 and i< size[1]:
			xvals.append(i)

	for i in range(yProbableNode-15, yProbableNode+15):
		if i>=0 and i< size[0]:
			yvals.append(i)		
	
	for y in yvals:
		#if xParentNode==None:
			#print('running')
		for x in xvals:
			if x== xProbableNode and y== yProbableNode:
				continue
			if nodeTree[y][x] == 1:
				probableRemaps.append([y,x])
				nodeDistance=  math.sqrt( (xProbableNode -x)*(xProbableNode-x) + (yProbableNode-y)*(yProbableNode-y) ) + costNode[y][x]
				#print(nodeDistance,lcstNodeDistance,costNode[y][x])
				if nodeDistance< lcstNodeDistance:
					lcstNodeDistance=nodeDistance
					xParentNode= x
					yParentNode= y

	return [yParentNode,xParentNode,lcstNodeDistance], probableRemaps				

	
###check for validity of point during obstacle check	
#checks for any obstacles
def notBlocked(ParentNode, x, y, img):

	flag=1

	if x-ParentNode[1]>0 :
		step =1
	else:
		step =-1

	for i in range (0, x-ParentNode[1], step):
		k=math.floor((i*y+(x-ParentNode[1]-i)*ParentNode[0])/(x-ParentNode[1]))
		for j in range(-1,2):
			if valid(k+j,i+ParentNode[1]):
				if notObstacle(k+j, i+ParentNode[1], img):
					continue
				else:
					flag=0
					return flag	

	return flag

def changeCost(parentNode, costNode, node):

	priorityQueue=[]
	heapq.heapify(priorityQueue)
	heapq.heappush(priorityQueue,[0, node])
	numberElementsPriority=[1]
	#print(parentNode[node[0]][node[1]])
	i=0
	while numberElementsPriority[i]:
		#print('struck\n')
		for j in range(0,numberElementsPriority[i]):
			numberElementsPriority.append(0)
			a=heapq.heappop(priorityQueue)
			distanceParent=math.sqrt((a[1][0]-parentNode[a[1][0]][a[1][1]][0][0])*(a[1][0]-parentNode[a[1][0]][a[1][1]][0][0])+(a[1][1]-parentNode[a[1][0]][a[1][1]][0][1])*(a[1][1]-parentNode[a[1][0]][a[1][1]][0][1]))
			costNode[a[1][0]][a[1][1]]= costNode[parentNode[a[1][0]][a[1][1]][0][0]][parentNode[a[1][0]][a[1][1]][0][1]] + distanceParent
			#print(len(parentNode[a[1][0]][a[1][1]]))
			#print(parentNode[a[1][0]][a[1][1]])
			for k in range(1,len(parentNode[a[1][0]][a[1][1]])):
				heapq.heappush(priorityQueue,[i+1, [parentNode[a[1][0]][a[1][1]][k][0], parentNode[a[1][0]][a[1][1]][k][1]]])
				numberElementsPriority[i+1] += 1
		i+=1		 

def changeColour(image, node1,node2):
	xDisplacement=node1[1]-node2[1]
	if xDisplacement>0 :
		step =1
	else:
		step =-1

	for i in range (0, xDisplacement, step):
		k=math.floor((i*node1[0]+(xDisplacement-i)*node2[0])/(xDisplacement))
		image.itemset((k,node2[1]+i,0),255)
		image.itemset((k,node2[1]+i,1),0)
		image.itemset((k,node2[1]+i,2),0)	

	yDisplacement=node1[0]-node2[0]
	if yDisplacement>0 :
		step =1
	else:
		step =-1

	for i in range (0, yDisplacement, step):
		j=math.floor((i*node1[1]+(yDisplacement-i)*node2[1])/(yDisplacement))
		image.itemset((node2[0]+i,j,0),255)
		image.itemset((node2[0]+i,j,1),0)
		image.itemset((node2[0]+i,j,2),0)						

def connectNodes(image, parentNode, node):
	xNode=node[1]
	yNode=node[0]
	print([yNode,xNode],parentNode[yNode][xNode][0])
	while [yNode,xNode]!= parentNode[yNode][xNode][0]:
		changeColour(image,[yNode,xNode],[parentNode[yNode][xNode][0][0],parentNode[yNode][xNode][0][1]])
		[yNode,xNode]= [parentNode[yNode][xNode][0][0],parentNode[yNode][xNode][0][1]]
		print([yNode,xNode],parentNode[yNode][xNode][0])
		
#takes a step in direction and creates node
def createNode(img, nodeTree, connectionTree, parentNode, costNode, listNode):###lst
	#print('running\n')
	global size

	d=0
	while not d: 
		xRad=random.randrange(0,size[1])
		yRad=random.randrange(0,size[0])
		#print(yRad,xRad)

		ClosestVertex=closestVertex(xRad, yRad, listNode)###lst
		#print(ClosestVertex)
		if ClosestVertex[2]!=0:
			xProbableNode = ClosestVertex[1] +int((xRad - ClosestVertex[1])*l/ClosestVertex[2])
			yProbableNode = ClosestVertex[0] +int((yRad - ClosestVertex[0])*l/ClosestVertex[2])
		d=	ClosestVertex[2]
	#print(yProbableNode, xProbableNode)

	if valid(xProbableNode, yProbableNode):
		if notObstacle(yProbableNode, xProbableNode, img):
			ParentNode, probableRemaps =lowestCostNode(nodeTree, xProbableNode, yProbableNode, costNode)
			#print(ParentNode)
			if notBlocked(ParentNode, xProbableNode, yProbableNode, img):
				nodeTree[yProbableNode][xProbableNode]=1
				if [ParentNode[0],ParentNode[1]]==[yProbableNode,xProbableNode]:
					print(error)
				parentNode[yProbableNode][xProbableNode][0]=[ParentNode[0],ParentNode[1]]
				parentNode[ParentNode[0]][ParentNode[1]].append([yProbableNode,xProbableNode])
				#print(parentNode[yProbableNode][xProbableNode], parentNode[ParentNode[0]][ParentNode[1]])
				#print(probableRemaps)
				###append in node lst
				listNode.append([yProbableNode,xProbableNode])
				###update the cost
				costNode[yProbableNode][xProbableNode]=ParentNode[2]
				###remap tree
				### if remap then conect tree
				remapTree( xProbableNode, yProbableNode, parentNode, costNode, probableRemaps)
				###if connect then
				connected, availableNodes = connectTree(xProbableNode, yProbableNode, connectionTree, img)
				return connected, availableNodes

	return 0, 0	
	
#checks for any remapping required
def remapTree( xNewNode, yNewNode, parentNode, costNode, probableRemaps):
	
	for node in probableRemaps:
		distanceNewNode=math.sqrt((node[0]-yNewNode)*(node[0]-yNewNode) + (node[1]-xNewNode)*(node[1]-xNewNode))
		#print(distanceNewNode,costNode[yNewNode][xNewNode],costNode[node[0]][node[1]])
		if costNode[yNewNode][xNewNode] + distanceNewNode < costNode[node[0]][node[1]]:
			remapped= 1
			#print(1)
			for child in parentNode[parentNode[node[0]][node[1]][0][0]][parentNode[node[0]][node[1]][0][1]]:
				if child[0]==node[0] and child[1]==node[1]:
					del child
					break

			parentNode[node[0]][node[1]][0]=[yNewNode, xNewNode]
			parentNode[yNewNode][xNewNode].append([node[0],node[1]])	
			changeCost(parentNode, costNode, node)

#checks for minimum distance between trees
def connectTree(xNewNode, yNewNode, connectionTree, img):
	global size

	availableNodes=[]
	connected=0
	nodeDistance=None

	for i in range(xNewNode-l, xNewNode+l+1):
		if i>=0 and i< size[1]:
			for j in range(yNewNode-l, yNewNode+l+1):
				if j>=0 and j< size[0]:
					if connectionTree[j][i]==1:
						nodeDistance=math.sqrt((xNewNode-i)*(xNewNode-i)+(yNewNode-j)*(yNewNode-j))
						if nodeDistance<=l:
							if notBlocked([yNewNode,xNewNode],i,j,img):
								availableNodes.append([[yNewNode, xNewNode], [j,i], nodeDistance])
								connected=1
	return  connected, availableNodes

def showTree(img, parentNewNode, parentNode, costNewNode,  costNode, availableConnections):
	totalPathLength=1000000000000000000000
	joint=None
	pathLength= None
	#print(availableConnections)
	for connection in availableConnections:
		pathLength=costNewNode[connection[0][0]][connection[0][1]] + costNode[connection[1][0]][connection[1][1]] + connection[2]
		if pathLength< totalPathLength:
			joint=connection
	
	print(joint)
	pathImage=img
	#print('call1')
	connectNodes(pathImage,parentNewNode, joint[0])
	#print('call2')
	connectNodes(pathImage,parentNode, joint[1])
	#print('call3')
	changeColour(pathImage,joint[0],joint[1])
	pathImage.itemset((joint[0][0],joint[0][1],0),255)
	pathImage.itemset((joint[0][0],joint[0][1],1),0)
	pathImage.itemset((joint[0][0],joint[0][1],2),0)
	print(pathLength)
	print(time.time()-startTime)
	cv2.namedWindow("window1",cv2.WINDOW_NORMAL)
	cv2.imshow('window1',pathImage)
	cv2.waitKey(1000)
	return pathLength, joint

def path(parentSourceTree, parentDestinationTree, costSourceNode, costDestinationNode, connection, pathLength):

	path = deque()
	stack = deque()
	xNode=connection[0][1]
	yNode=connection[0][0]
	stack.append([yNode,xNode])
	numberNodes=1

	while parentSourceTree[yNode][xNode][0]!=[xNode,yNode]:
		
		xNode=parentSourceTree[yNode][xNode][0][1]
		yNode=parentSourceTree[yNode][xNode][0][0]
		stack.append([yNode,xNode])
		numberNodes+=1

	startPoint=	stack.pop()
	for i in range(0,numberNodes-1):
		endPoint=stack.pop()
		theta=math.atan((endPoint[0]-startPoint[0])/(endPoint[1]-startPoint[1]))
		error= (pathLength- costSourceNode[startPoint[0]][startPoint[1]])*11/600
		path.append([(600-startPoint[0])*11/600,startPoint[1]*11/600,theta,error])
		startPoint=endPoint

	endPoint=connection[1]
	theta=math.atan((endPoint[0]-startPoint[0])/(endPoint[1]-startPoint[1]))
	error= (pathLength- costSourceNode[startPoint[0]][startPoint[1]])*11/600
	path.append([(600-startPoint[0])*11/600,startPoint[1]*11/600,theta,error])
	startPoint=endPoint	

	while parentSourceTree[startPoint[0]][startPoint[1]][0]!=startPoint:	
		endPoint=parentSourceTree[startPoint[0]][startPoint[1]][0]
		theta=math.atan((endPoint[0]-startPoint[0])/(endPoint[1]-startPoint[1]))
		error= (costSourceNode[startPoint[0]][startPoint[1]])*11/600
		path.append([(600-startPoint[0])*11/600,startPoint[1]*11/600,theta,error])
		startPoint=endPoint
		numberNodes+=1
	path.append([(600-startPoint[0])*11/600,startPoint[1]*11/600,0,0])
	numberNodes+=1
	turtleMove(path, numberNodes)	


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
	teleport(startpoint[1],startpoint[0],0)
	velocity= Twist()

	for i in range(0,numberNodes-1):

		endPoint=path.popleft()
		velocity.linear.x=0
		velocity.linear.y=0
		velocity.linear.z=0
		velocity.angular.x=0
		velocity.angular.y=0
		velocity.angular.z=startPoint[2]
		velocityPublisher.publish(velocity)
		time.sleep(1)

		while (position[0]!=endPoint[1] or position[1]!=endPoint[0]):

			rospy.Subscriber("/turtle1/pose", Pose, stop)
			velocity.linear.x=startPoint[4]
			velocity.linear.y=0
			velocity.linear.z=0
			velocity.angular.x=0
			velocity.angular.y=0
			velocity.angular.z=0

		startpoint=endPoint		

def main():
	global size
	global xSource
	global ySource
	global xDestination
	global yDestination

	img= cv2.imread("obstacle.png",1)
	print(img.shape[0])
	size.append(img.shape[0])
	size.append(img.shape[1])

	#keeps tracks of points which are node
	nodeSourceTree=[[0 for _ in range(size[1])] for _ in range(size[0])]
	nodeDestinationTree=[[0 for _ in range(size[1])] for _ in range(size[0])]

	###create lst of nodes
	listSourceNode=[]
	listDestinationNode=[]


	#stores parent and child of a node on source and destination tree nodes
	parentSourceNode=[[[[y, x]] for x in range(size[1])] for y in range(size[0])]
	parentDestinationNode=[[[[y, x]] for x in range(size[1])] for y in range(size[0])]

	#cost array for source and destination tree nodes
	costSourceNode=[[100000 for _ in range(size[1])] for _ in range(size[0])]
	costDestinationNode=[[100000 for _ in range(size[1])] for _ in range(size[0])]

	sourceDestinationPoints(img)
	sourcePoint=[ySource,xSource]
	destinationPoint=[yDestination,xDestination]

	#print(sourcePoint,destinationPoint)
	nodeSourceTree[sourcePoint[0]][sourcePoint[1]]=1
	nodeDestinationTree[destinationPoint[0]][destinationPoint[1]]=1

	listSourceNode.append([sourcePoint[0],sourcePoint[1]])
	listDestinationNode.append([destinationPoint[0],destinationPoint[1]])

	costSourceNode[sourcePoint[0]][sourcePoint[1]]=0
	costDestinationNode[destinationPoint[0]][destinationPoint[1]]=0

	connected=0
	connection=None
	pathLength=None
	while not connected:
		#print('calls')
		connected, availableConnections=createNode(img, nodeSourceTree, nodeDestinationTree, parentSourceNode, costSourceNode, listSourceNode)###pass list of nodes
		if connected:
			connection=showTree(img, parentSourceNode, parentDestinationNode, costSourceNode,  costDestinationNode, availableConnections)
			break
		#print('calld')
		connected, availableConnections=createNode(img, nodeDestinationTree, nodeSourceTree, parentDestinationNode, costDestinationNode, listDestinationNode)###pass list of nodes
		if connected:
			pathLength, connection=showTree(img, parentDestinationNode, parentSourceNode, costDestinationNode,  costSourceNode, availableConnections)
			node= connection[0]
			connection[0]= connection[1]
			connection[1]= node
			break

	#print(connected, availableConnections)		
	print(connection, pathLength)
	path(parentSourceNode, parentDestinationNode, costSourceNode, costDestinationNode, connection, pathLength)		

if __name__ == "__main__":			
	
	main()