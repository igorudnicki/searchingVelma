#!/usr/bin/env python

import roslib
import rospy
import numpy as numpy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Kinect(object):

	def __init__(self):
		rospy.loginfo("Initiating Kinect camera...")
		self.rgb_camera_topic = "/head_kinect/rgb/image_raw"
		self.camera_info_topic = "/head_kinect/rgb/camera_info"
		self.cvbridge = CvBridge()

		self.displayRgbTopic()
		self.displayCameraInfoTopic()
		self.checkIfCameraReady()
		self.checkIfCameraInfoReady()

		rospy.loginfo("Initiating camera subscribers...")
		self.image_sub = rospy.Subscriber(self.rgb_camera_topic,Image,self.camera_callback)
		self.cinfo_sub = rospy.Subscriber(self.camera_info_topic,CameraInfo,self.camera_info_callback)
		rospy.loginfo("Camera is ready to use")

	def checkIfCameraReady(self):

		self.camera_capture = None
		rospy.loginfo("Checking if camera ready...")
		self.testnr=0
		while self.camera_capture is None and self.testnr<=5:
			msg = rospy.wait_for_message(self.rgb_camera_topic, Image, timeout=5.0)
			self.camera_capture = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
			self.testnr=self.testnr+1
		if self.camera_capture is None:
			rospy.logerr("Problem with rgb camera initialization")

	def checkIfCameraInfoReady(self):

		self.camera_info = None
		rospy.loginfo("Checking if camera info ready...")
		self.testnr=0
		while self.camera_info is None and self.testnr<=5:
			self.camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo, timeout=5.0)
			self.testnr=self.testnr+1		
		if self.camera_info is None:
			rospy.logerr("Problem with camera info")

	def showPathImg(self,img_path,time_ms=2000):

		image = cv2.imread(img_path)   
		window_name = 'ImgFromPath'
		cv2.imshow(window_name, image) 
		cv2.waitKey(time_ms)   
		cv2.destroyAllWindows()     

	def showLastImg(self,time_ms=2000):
		window_name = "LastCameraCapture"
		cv2.imshow(window_name,self.camera_capture) 
		cv2.waitKey(time_ms)   
		cv2.destroyAllWindows()

	def displayRgbTopic(self):
		rospy.loginfo("Camera topic is "+ str(self.rgb_camera_topic))

	def displayCameraInfoTopic(self):
		rospy.loginfo("Camera topic is "+ str(self.camera_info_topic))

	def camera_callback(self,msg):      
		#bgr8 default OpenCv encoding
		self.camera_capture = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

	def camera_info_callback(self,msg):
		self.camera_info=msg
		if self.camera_info is not None:
			self.K = numpy.matrix(numpy.reshape(self.camera_info.K, (3, 3)))
			self.P = numpy.matrix(numpy.reshape(self.camera_info.P, (3, 4)))

	def getCameraInfo(self):
		if self.camera_info is not None:
			return self.camera_info
	def getWidthHeight(self):
		if self.camera_info is not None:	
			return self.camera_info.width,self.camera_info.height
	def getLastCameraCapture(self):
		if self.camera_capture is not None:
			return self.camera_capture

	def resize_image(self, img_path, percentage=50):
		image = cv2.imread(img_path)   
		width = int(image.shape[1] * percentage / 100)
		height = int(image.shape[0] * percentage/ 100)
		dim = (width, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		print('Resized Dimensions : ',resized_image.shape)
		return resized_image

	def resize_image_num(self,img_path,sq_side=128):
		image = cv2.imread(img_path)   
		width = sq_side
		height = sq_side
		dim = (width, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		return resized_image

	def getShape(self,img_path):
		image = cv2.imread(img_path)   
		print('Resized Dimensions : ',image.shape[1],image.shape[0])
		return image.shape[1],image.shape[0]
	
	def getShapeRaw(self,cvimg):
		return cvimg.shape[1],cvimg.shape[0]

	def resizeImageRaw(self,cvimg,sq_side=128):
		width = sq_side
		height = sq_side
		dim = (width, height)
		resized_image = cv2.resize(cvimg, dim, interpolation = cv2.INTER_AREA)
		return resized_image

if __name__ == "__main__":
	rospy.init_node('kinect_node',anonymous=True)
	kinect = Kinect()
	#cv2.imshow("ResizedImg",kinect.resize_image("/home/igor/Pulpit/m.png",20)) 

	rospy.spin()