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
from train_cnn1 import CNN_Network
from camera_kinect_rgb import Kinect
from keras.applications.mobilenet_v2 import*

from keras.preprocessing import image

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


class Classifier(object):

    def __init__(self):

        rospy.loginfo("Starting Unit Tester for Empty Test...")
        self.kinect = Kinect()
        self.package_path = "/home/igor/mobile/src/igorudnicki/"
        self.weights_path = self.package_path + "model_weight_checkpoints_gen_empty/model0.0000024456.h5"
        self.initiateModel()

    def initiateModel(self):
        self.cnn = CNN_Network()
        self.cnn.model.load_weights(self.weights_path)
        rospy.loginfo("Weights loaded")
        
    def predict_image(self,cv2_img=None, path=None):

        path = "/home/igor/mobile/src/igorudnicki/temp_imgs/temp.png"

        camera_capture = self.kinect.getLastCameraCapture()
        camera_capture = self.kinect.resizeImageRaw(camera_capture)
        cv2.imwrite(path,camera_capture)

        test_image = image.load_img(path, target_size = (128, 128))
        test_image = image.img_to_array(test_image)
        test_image = np.expand_dims(test_image, axis = 0)
        result = self.cnn.model.predict(test_image)

        print(result)

        if result[0][0] == 1:
            prediction = 'Object not detected'
        else:
            prediction = 'Object detected'
    
        return prediction

if __name__ == "__main__":

    rospy.init_node('unit_test_node',anonymous=True)
    rospy.spin()        
