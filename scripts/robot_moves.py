#!/usr/bin/env python
import roslib
import rospy
import math

from velma_common import*
from rcprg_ros_utils import exitError
from geometry_msgs.msg import Twist,PoseStamped,Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class Position(object):

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta 

class RobotMover(object):

	def __init__(self,velma):

		rospy.loginfo("Initiating Robot Mover...")
		self.velma=velma
		self.pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=10)
		self.currentPos=Position(0,0,0)
		self.gazeboPos = Position(0,0,0)
		self.index = 0
		self.runOnce = True
		self.listener()

	def rotateToDest(self,xdest,ydest,rotspeed= 0.35,freq=250):

		rospy.loginfo("Rotating...")
		tol=0.01
		msg = Twist()
		rate = rospy.Rate(freq)

		angle = math.atan2(ydest - self.gazeboPos.y, xdest - self.gazeboPos.x)
		if((angle-self.gazeboPos.theta)<0):
			rotspeed = -rotspeed
		while math.fabs(angle-self.gazeboPos.theta)>tol:
			msg.angular.z= rotspeed
			self.pub.publish(msg)
			rate.sleep()
		rospy.loginfo("Rotation end")
		msg.angular.z = 0.0
		self.pub.publish(msg)

	def driveFront(self,xdest,linearspeed=0.75,freq=100):

		msg = Twist()
   		tol=0.01
		rate = rospy.Rate(freq)
		ydest = 0.0

		lastDistance = math.sqrt(math.pow(ydest- self.gazeboPos.y, 2) + math.pow(xdest - self.gazeboPos.x, 2))
		if (self.currentPos.x>xdest):
			linearspeed = -linearspeed 
		while True:
			msg.linear.x = linearspeed  
			currentDistance = math.sqrt(math.pow(ydest- self.gazeboPos.y, 2) + math.pow(xdest - self.gazeboPos.x, 2))
			if lastDistance  - currentDistance < -tol:
				break
			self.pub.publish(msg)
			lastDistance = currentDistance
			rate.sleep()

		msg.linear.x = 0.0
		self.pub.publish(msg)

	def driveSides(self,xdest,linearspeed=0.75,freq=250):

		msg = Twist()
		tol=0.01
		rate = rospy.Rate(freq)
		T_M_B = self.velma.getTf("map","B")

		ydest =  T_M_B.p[1]

		lastDistance = math.sqrt(math.pow(ydest- self.gazeboPos.y, 2) + math.pow(xdest - self.gazeboPos.x, 2))

		while True:
			msg.linear.y = linearspeed  
			currentDistance = math.sqrt(math.pow(ydest- self.gazeboPos.y, 2) + math.pow(xdest - self.gazeboPos.x, 2))
			if lastDistance  - currentDistance < -tol:
				break
			self.pub.publish(msg)
			lastDistance = currentDistance
			rate.sleep()

		msg.linear.y = 0.0
		self.pub.publish(msg)


	def listener(self): 
		rospy.Subscriber('/odom', Odometry, self.getOdomNav)
		rospy.Subscriber('/gazebo/model_states',ModelStates, self.getGazeboNav) 

	def getOdomNav(self,data):
	
		self.currentPos.x = data.pose.pose.position.x
		self.currentPos.y = data.pose.pose.position.y         
		rotation = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
		[roll,pitch,yaw] = rotation.GetRot()
		self.currentPos.theta= yaw;

	def getGazeboNav(self,data):

		if (self.runOnce == True):
			self.index = self.getVelmaIndex(data)
			self.runOnce = False

		self.gazeboPos.x = data.pose[self.index].position.x
		self.gazeboPos.y = data.pose[self.index].position.y
		self.gazeboPos.z = data.pose[self.index].position.z
				
		rotation = PyKDL.Rotation.Quaternion(data.pose[self.index].orientation.x, data.pose[self.index].orientation.y, data.pose[self.index].orientation.z, data.pose[self.index].orientation.w)
		[roll,pitch,yaw] = rotation.GetRot()
		self.gazeboPos.theta= yaw;


	def getVelmaIndex(self,data):

		for model in data.name:
			if model == "velma":
				return self.index
			self.index = self.index+1


if __name__ == "__main__":

	rospy.init_node('robot_mover',anonymous=True)
	rospy.spin()
