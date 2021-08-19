#!/usr/bin/env python
import roslib
import rospy
import PyKDL
import math
import utils

from searcher.srv import *
from velma_common import*
from rcprg_ros_utils import exitError
from geometry_msgs.msg import Twist,PoseStamped,Pose,Point,Quaternion

#from test import Classifier

sys.path.append('/home/igor/mobile/src/igorudnicki/cnnscripts')

class Detector(object):

	def __init__(self,velma):

		rospy.loginfo("Initiating Detector...")
		self.velma=velma
		#self.classifier = Classifier()


	def classify_cnn(self):
		
		print("Classifying...")
		#prediction = self.classifier.predict_image()
		#print(prediction)

	def segment_ransac(self):
		resp=self.SegmentationClient()
		if resp.x ==0:
			return "RANSAC could not segment cylinder"
		else:
			print("X: ",resp.x)
			print("Y: ",resp.y)
			print("Z: ",resp.z)
			return "Cylinder found"

	def SegmentationClient(self):
		rospy.wait_for_service('cylinder_segmentation')
		try:
			cylinder_segmentation = rospy.ServiceProxy('cylinder_segmentation',Segment)
			resp = cylinder_segmentation()
			return resp
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e) 

def detection_callback(req):

	if req.mode==0: 
		result = det.segment_ransac()
		return result

	elif req.mode==1:                
		return "Mode 1 completed"


if __name__ == "__main__":

	rospy.init_node('detection_node',anonymous=True)
	velma=VelmaInterface()
	det = Detector(velma)
	s = rospy.Service('detection', Detect, detection_callback)
	print(det.segment_ransac())

	rospy.spin()