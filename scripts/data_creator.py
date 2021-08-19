#!/usr/bin/env python

import roslib
import rospy
import numpy as numpy
import sys
import shutil
import os
import math
from datetime import date
import time
import cv2
import numpy as np
from model_pose import GazeboModel
from camera_kinect_rgb import Kinect 
from csv_gen import CSVGenerator

class DataGenerator(object):

	def __init__(self):
		rospy.loginfo("Initiating Dataset Generator...")
		self.kinect = Kinect()
		self.csvg = CSVGenerator()
		self.date = self.getDate()
		self.package_path = "/home/igor/mobile/src/igorudnicki/"
		self.createImageFoler()
		self.createImageEmptyFolder()
		self.createCSVFolder()

	def createImageFoler(self):
		self.images_dir = os.path.join(self.package_path, 'images/')
		if not os.path.exists(self.images_dir):
			os.makedirs(self.images_dir)
		rospy.loginfo("Image folder created")

	def createImageEmptyFolder(self):
		self.images_empty_dir = os.path.join(self.package_path, 'images_empty/')
		if not os.path.exists(self.images_empty_dir):
			os.makedirs(self.images_empty_dir)
		rospy.loginfo("Image folder created")

	def createCSVFolder(self):
		self.csv_dir = os.path.join(self.package_path, 'csv/')
		if not os.path.exists(self.csv_dir):
			os.makedirs(self.csv_dir)
		rospy.loginfo("CSV folder created")

	def getDate(self):

		today = date.today()
		date_today = today.strftime("%d.%m.%Y")
		rospy.loginfo(date_today)
		return date_today

	def generateDataEmpty(self):

		rate = rospy.Rate(1)
		index_img=501
		while not rospy.is_shutdown():
			camera_capture = self.kinect.getLastCameraCapture()
			rospy.loginfo("Image taken correctly")
			img_name = str(self.date)+ "."+str(index_img)
			img_ext = ".png"
			img_full_name = img_name + img_ext
			path_img_file = "/home/igor/mobile/src/igorudnicki/images_empty/" + img_full_name        #images_empty
			cv2.imwrite(path_img_file,camera_capture)
			self.csvg.appendFullFileEmptyTable(path_img_file)
			index_img += 1
			rospy.sleep(0.5)
			rate.sleep()

	def generateData(self):
		rospy.loginfo("Starting generation...")
		index_img=1
		camera_width,camera_height = self.kinect.getWidthHeight()
		rate = rospy.Rate(1)
		manipulated_model = GazeboModel("jar")
		rospy.sleep(1.0)          #wait until the model pose is not None
		last_model_pose = manipulated_model.model_pose

		while not rospy.is_shutdown():
			manipulated_model = GazeboModel("jar")
			camera_capture = self.kinect.getLastCameraCapture()
			rospy.sleep(1.0)
			if camera_capture is None:
			 	rospy.logwarn("Camera capture was none")
				image = numpy.zeros((cam_info.height, cam_info.width))
			else:
				pose_diff = math.sqrt((last_model_pose.position.x-manipulated_model.model_pose.position.x)**2+(last_model_pose.position.y-manipulated_model.model_pose.position.y)**2+(last_model_pose.position.z-manipulated_model.model_pose.position.z)**2)
				print(pose_diff)
				if math.fabs(pose_diff)<0.001:
					rospy.loginfo("Image not taken . Model position didnt change")	
					print("CHANGE NOW IN 3.5 SECS")
					rospy.sleep(3.5)
					continue
				else:
					rospy.loginfo("Image taken correctly")
					img_name = str(self.date)+ "."+str(index_img)
					img_ext = ".png"
					img_full_name = img_name + img_ext
					path_img_file = "/home/igor/mobile/src/igorudnicki/images/" + img_full_name        #images_empty
					cv2.imwrite(path_img_file,camera_capture)
					self.csvg.appendFullFile(path_img_file)
					index_img += 1

			last_model_pose = manipulated_model.model_pose
			print("CHANGE NOW IN 3.5 SECS")
			rospy.sleep(3.5)  #wait until the world change
			rate.sleep()

if __name__ == "__main__":
	rospy.init_node('dataset_gen_node',anonymous=True)
	dsg = DataGenerator()
	dsg.generateDataEmpty()
	rospy.spin()



