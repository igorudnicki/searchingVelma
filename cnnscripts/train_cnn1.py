#!/usr/bin/env python

import rospy
import rospkg
import time
import math
import cv2
import tensorflow as tf
import numpy as np
import sys
sys.path.append('/home/igor/mobile/src/igorudnicki/scripts')

from keras import Model
from keras.utils import plot_model
from keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau, TensorBoard
from keras.layers import MaxPooling2D, Conv2D, Reshape
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D,MaxPool2D
from keras.layers import Activation, Dropout, Flatten, Dense

from camera_kinect_rgb import Kinect
from keras.preprocessing.image import ImageDataGenerator

from keras.utils import Sequence
from keras.optimizers import Adam

from keras.applications.mobilenet_v2 import*

import keras.backend as K
import os
import shutil
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

class CNN_Network(object):

    def __init__(self):

        rospy.loginfo("Convultional Neural Network initializer...")
        self.model = None
        self.img_size = 128
        self.createModel(self.img_size)
       

    def createModel(self,size):
        self.model = Sequential()#add model layers
        self.model.add(Conv2D(16, kernel_size=3, activation='relu', input_shape=(size,size,3)))
        self.model.add(MaxPool2D(pool_size=(2,2)))
        convultion_filters_list = [32,64,64,64]

        for filter_size in convultion_filters_list:
            self.model.add(Conv2D(filter_size, kernel_size=3, activation='relu'))
            self.model.add(MaxPool2D(pool_size=(2,2)))

        self.model.add(Flatten())
        self.model.add(Dense(512,activation = 'relu'))
        #self.model.add(Dropout(0.5))
        self.model.add(Dense(1,activation = 'sigmoid'))
        #model.summary()
        #plot_model(self.model, to_file='./model2.png', show_shapes=True) 

class CNN_Trainer(object):

    def __init__(self,model):

        rospy.loginfo("CNN Trainer initializing...")
        self.model = model
        self.kinect = Kinect()
        self.train_datagen = None
        self.validation_datagen = None
        self.package_path = "/home/igor/mobile/src/igorudnicki/"



    def initiateTraining(self,epochs, batch_size, patience, threads,load_weight_starting_file, initial_learning_rate=0.0001, min_learning_rate = 1e-8):

        self.createLogsCheckpointsFolders()
    
        self.train_datagen = ImageDataGenerator(rescale = 1./255)
        self.validation_datagen = ImageDataGenerator(rescale = 1./255)
        self.train_datagen = self.train_datagen.flow_from_directory('/home/igor/mobile/src/igorudnicki/training_binary/training_set',
                                                 target_size = (128, 128),
                                                 batch_size = batch_size,
                                                 class_mode = 'binary')

        self.validation_datagen = self.validation_datagen.flow_from_directory('/home/igor/mobile/src/igorudnicki/training_binary/validation_set',
                                            target_size = (128, 128),
                                            batch_size = batch_size,
                                            class_mode = 'binary')

        self.showClassesBinary()

        adam_optim = Adam(lr=initial_learning_rate, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
        self.model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

        full_model_file_name = "model{loss:.10f}.h5"
        model_file_path = os.path.join(self.checkpoints_folder, full_model_file_name)
        checkpoint = ModelCheckpoint(model_file_path, monitor="loss", verbose=1, save_best_only=True,save_weights_only=True, mode="auto", period=1)
    
        stop = EarlyStopping(monitor="loss", patience=patience*3, mode="auto")
        reduce_lr = ReduceLROnPlateau(monitor="loss", factor=0.2, patience=patience, min_lr=min_learning_rate, verbose=1, mode="auto")
        tensorboard_clb = TensorBoard(log_dir=self.logs_folder, histogram_freq=0,write_graph=True, write_images=True)


        self.model.fit_generator(generator=self.train_datagen,        #pobiera batche w rozmiarze 32 , az skoncza sie dane trenujace
                            epochs=epochs,
                        validation_data=self.validation_datagen,
                        callbacks=[checkpoint, reduce_lr, stop, tensorboard_clb],
                        workers=threads,        #sprawdzic ile threads
                        use_multiprocessing=True,
                        shuffle=True,
                        verbose=1)

    
        capture = self.kinect.getLastCameraCapture()
        self.kinect.showLastImg()
        capture_resized = self.kinect.resizeImageRaw(capture)


        model_input = np.array(capture_resized, dtype='f')
        model_input = preprocess_input(model_input)    
        model_input = np.expand_dims(model_input,axis=0) 


        result = self.model.predict(model_input)
        print(result)

        if result[0][0] == 1:
            prediction = 'Table is empty'
        else:
            prediction = 'There is something on the table'
    
        return prediction
    def showClassesBinary(self):

        if self.train_datagen != None:
            print(self.train_datagen.class_indices)
        else:
            print("There is not yet any training data")

    def readTensorFlowFile(self):

        events_file_path= self.package_path + "logs_tensor_empty/...."
        for e in tf.train.summary_iterator(events_file_path):
            for v in e.summary.value:
                if v.tag == 'loss' or v.tag == 'accuracy':
                    print(v.simple_value) 

    def createLogsCheckpointsFolders(self):

        self.checkpoints_folder = os.path.join(self.package_path, "model_weight_checkpoints_gen_empty")
        self.logs_folder = os.path.join(self.package_path, "logs_tensor_empty")
      
        # We clean up the training folders
        if os.path.exists(self.checkpoints_folder):
            shutil.rmtree(self.checkpoints_folder)
        os.makedirs(self.checkpoints_folder)
        print("Created folder " + str(self.checkpoints_folder))
    
        if os.path.exists(self.logs_folder):
            shutil.rmtree(self.logs_folder)
        os.makedirs(self.logs_folder)
        print("Created folder " + str(self.logs_folder))


if __name__ == "__main__":

    rospy.init_node('training_2_node', anonymous=True, log_level=rospy.WARN)

    image_size = 128
    epochs = 25
    batch_size = 32
    patience = 5
    threads = 2 
    initial_learning_rate = 0.0001
    min_learning_rate = 0.00000001
    load_weight_starting_file = None

    cnn = CNN_Network()
   # rospy.sleep(0.5)
    if load_weight_starting_file:
        rospy.logwarn("Preload Weights "+str(load_weight_starting_file))
        model.load_weights(load_weight_starting_file)
    else:
        rospy.logerr("Starting from empty weights") 

    cnn_trainer = CNN_Trainer(cnn.model)
    cnn_trainer.initiateTraining(epochs,batch_size,patience,threads,load_weight_starting_file,initial_learning_rate,min_learning_rate)