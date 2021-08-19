#!/usr/bin/env python

import roslib
import rospy
from gazebo_msgs.msg import ModelStates


class GazeboModel(object):

	def __init__(self,model_nm):
		rospy.loginfo("Initiating Gazebo Model Pose...")

		self.index = 0
		self.runOnce = True
		self.model_name = model_nm
		self.model_pose = None
		self.gazebo_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

	def callback(self,data):

		if (self.runOnce == True):
			self.index = self.getModelIndex(self.model_name,data)
			self.runOnce = False
			self.model_pose = self.getModelPose(self.index,data)

	def getModelIndex(self,name,data):

		for model in data.name:
			if model == name:
				return self.index
			self.index = self.index+1

	def getModelPose(self,idx,data):

		return data.pose[idx]

if __name__ == "__main__":

	rospy.init_node('model_pose_node',anonymous=True)
	model = GazeboModel("jar")

	rospy.spin()
