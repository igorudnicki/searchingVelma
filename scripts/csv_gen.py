#!/usr/bin/env python

import roslib
import rospy
import csv
import os
import shutil
import cv2
from cv_bridge import CvBridge, CvBridgeError
from model_pose import GazeboModel
from camera_kinect_rgb import Kinect

class CSVGenerator(object):

	def __init__(self):
		rospy.loginfo("Initiating CSV File Generator...")
		self.package_path = "/home/igor/mobile/src/igorudnicki/"
		self.splitting_param = 9
		self.kinect = Kinect()
		#used to append images folder with all photos

	def appendFullFile(self,path):
		rospy.loginfo("Appending existing full csv file....")
		with open('/home/igor/mobile/src/igorudnicki/csv/detected.csv', mode='a') as png_poses_file:
			csvwriter = csv.writer(png_poses_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			csvwriter.writerow([path])	
	
	def appendFullFileEmptyTable(self,path):
		rospy.loginfo("Appending existing full csv file....")
		with open('/home/igor/mobile/src/igorudnicki/csv/empty_table.csv', mode='a') as png_poses_file:
			csvwriter = csv.writer(png_poses_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
			csvwriter.writerow([path])	
			
	#creation of training and validation folders
	def createTrainingFoler(self,empty=None):
		if empty ==None:
			self.training_dir = os.path.join(self.package_path, 'training/')
			if os.path.exists(self.training_dir):
				shutil.rmtree(self.training_dir)
				os.makedirs(self.training_dir)
				rospy.loginfo("Deleted and new training folder created")
			else:
				os.makedirs(self.training_dir)
				rospy.loginfo("New training folder created")
		else: 
			self.training_dir = os.path.join(self.package_path, 'training_empty/')
			if os.path.exists(self.training_dir):
				shutil.rmtree(self.training_dir)
				os.makedirs(self.training_dir)
				rospy.loginfo("Deleted and new training folder created")
			else:
				os.makedirs(self.training_dir)
				rospy.loginfo("New training folder created")

	def createValidationFoler(self,empty=None):
		if empty ==None:
			self.validation_dir = os.path.join(self.package_path, 'validation/')
			if os.path.exists(self.validation_dir):
				shutil.rmtree(self.validation_dir)
				os.makedirs(self.validation_dir)
				rospy.loginfo("Deleted and new validation folder created")
			else:
				os.makedirs(self.validation_dir)
				rospy.loginfo("New validation folder created")
		else:
			self.validation_dir = os.path.join(self.package_path, 'validation_empty/')
			if os.path.exists(self.validation_dir):
				shutil.rmtree(self.validation_dir)
				os.makedirs(self.validation_dir)
				rospy.loginfo("Deleted and new validation folder created")
			else:
				os.makedirs(self.validation_dir)
				rospy.loginfo("New validation folder created")			

	def appendTrainFile(self,path,img_width=None,img_height=None,m_pose=None,model_name=None,class_zero=None):
		rospy.loginfo("Appending training csv file....")
		if img_width != None:
			with open('/home/igor/mobile/src/igorudnicki/csv/training.csv', mode='a') as training_file:
				csvwriter = csv.writer(training_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				csvwriter.writerow([path,img_width,img_height,m_pose[0],m_pose[1],m_pose[2],m_pose[3],m_pose[4],m_pose[5],m_pose[6],model_name,class_zero])
		else:
			with open('/home/igor/mobile/src/igorudnicki/csv/training_empty.csv', mode='a') as training_file:
				csvwriter = csv.writer(training_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				csvwriter.writerow([path])

	def appendValidationFile(self,path,img_width=None,img_height=None,m_pose=None,model_name=None,class_zero=None):
		rospy.loginfo("Appending validation csv file....")
		if img_width != None:
			with open('/home/igor/mobile/src/igorudnicki/csv/validation.csv', mode='a') as validation_file:
				csvwriter = csv.writer(validation_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				csvwriter.writerow([path,img_width,img_height,m_pose[0],m_pose[1],m_pose[2],m_pose[3],m_pose[4],m_pose[5],m_pose[6],model_name,class_zero])
		else:
			with open('/home/igor/mobile/src/igorudnicki/csv/validation_empty.csv', mode='a') as validation_file:
				csvwriter = csv.writer(validation_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				csvwriter.writerow([path])

	def createTrainingValidationData(self,empty=None):	
		rospy.loginfo("Creating training and validation data")
		if empty==None:
			self.createTrainingFoler()
			self.createValidationFoler()
			idx=0
			with open('/home/igor/mobile/src/igorudnicki/csv/detected.csv', mode='r') as f:
				reader = csv.reader(f)
				data = list(reader)
			for element in data:
				starting_path = element[0]
				width = element[1]
				height = element[2]
				model_name = element[10]
				class_zero = element[11]
				pose = [element[3],element[4],element[5],element[6],element[7],element[8],element[9]]
				img_name = starting_path[41:]
				if idx%self.splitting_param==0:
					target = "/home/igor/mobile/src/igorudnicki/validation/" + str (img_name)
					shutil.copyfile(starting_path, target) 
					cap = self.kinect.resize_image_num(target)  ## resize image to 128x128 and save again
					cv2.imwrite(target,cap)
					self.appendValidationFile(target,width,height,pose,model_name,class_zero)
				else:
					target = "/home/igor/mobile/src/igorudnicki/training/" + str (img_name)
					shutil.copyfile(starting_path, target) 
					cap = self.kinect.resize_image_num(target)  ## resize image to 128x128 and save again
					cv2.imwrite(target,cap)
					self.appendTrainFile(target,width,height,pose,model_name,class_zero)
				idx=idx+1
		else:
			self.createTrainingFoler(1)
			self.createValidationFoler(1)
			idx=0
			with open('/home/igor/mobile/src/igorudnicki/csv/empty_table.csv', mode='r') as f:
				reader = csv.reader(f)
				data = list(reader)
			for element in data:
				starting_path = element[0]
				img_name = starting_path[47:]
				if idx%self.splitting_param==0:
					target = "/home/igor/mobile/src/igorudnicki/validation_empty/" + str (img_name)
					shutil.copyfile(starting_path, target) 
					cap = self.kinect.resize_image_num(target)  ## resize image to 128x128 and save again
					cv2.imwrite(target,cap)
					self.appendValidationFile(target)
				else:
					target = "/home/igor/mobile/src/igorudnicki/training_empty/" + str (img_name)
					shutil.copyfile(starting_path, target) 
					cap = self.kinect.resize_image_num(target)  ## resize image to 128x128 and save again
					cv2.imwrite(target,cap)
					self.appendTrainFile(target)
				idx = idx +1

if __name__ == "__main__":

	rospy.init_node('csv_gen_node',anonymous=True)
	csvg = CSVGenerator()
	csvg.createTrainingValidationData(1)
