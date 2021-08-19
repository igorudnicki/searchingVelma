#!/usr/bin/env python

import rospy
import rospkg
import csv
import math
import cv2
import numpy as np
import os
import sys
sys.path.append('/home/igor/mobile/src/igorudnicki/scripts')
from train_cnn2 import createModel
from keras.applications.mobilenet_v2 import*


os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from model_pose import GazeboModel
from camera_kinect_rgb import Kinect

class UnitTester(object):

    def __init__(self):

        rospy.loginfo("Starting Unit Tester...")
        self.model = GazeboModel("jar")
        self.kinect = Kinect()
        self.package_path = "/home/igor/mobile/src/igorudnicki/"
        self.model_path = self.package_path + "model_weight_checkpoints_gen/model0.0155.h5"
        rospy.sleep(1.0)
        self.real_model_x,self.real_model_y = self.getModelPositionXY()
        print("Model position [X] is ",self.real_model_x)
        print("Model position [Y] is ",self.real_model_y)

    def getModelPositionXY(self):
        return self.model.model_pose.position.x,self.model.model_pose.position.y
 
    def initiateModel(self):
        self.model = createModel(128, 1.0, 2)
        self.model.load_weights(self.model_path)
        rospy.loginfo("Weights loaded")
        
    def predict_image(self,cv2_img=None, path=None):

        if path == None:
            self.initiateModel()
            capture = self.kinect.getLastCameraCapture()
            self.kinect.showLastImg()
            capture_resized = self.kinect.resizeImageRaw(capture)
        else:
            self.initiateModel()
            capture_path = cv2.imread(path)
            self.kinect.showPathImg(path)
            capture_resized = self.kinect.resizeImageRaw(capture_path)
    
        model_input = np.array(capture_resized, dtype='f')
        model_input = preprocess_input(model_input)
        
        prediction = self.model.predict(x=np.array([model_input]))[0]
    
        return prediction

if __name__ == "__main__":
    rospy.init_node('unit_test_node',anonymous=True)

    utc = UnitTester()
    prediction = utc.predict_image()
    print(prediction)
    rospy.spin()        