#!/usr/bin/env python

import rospy
import rospkg
import time
import csv
import math
import cv2
import tensorflow as tf
import numpy as np
from keras import Model
from keras.applications.mobilenet_v2 import*
from keras.utils import plot_model
from keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau, TensorBoard
from keras.layers import MaxPooling2D, Conv2D, Reshape
from keras.utils import Sequence
from keras.optimizers import Adam

import keras.backend as K
import os
import shutil
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

class DataSequence(Sequence):

    def __load_images(self, dataset):
        return np.array([cv2.imread(f) for f in dataset], dtype='f')

    def __init__(self, csv_file, batch_size=32, output_size=2):
        
        self.output_size = output_size   
        self.paths = []
        self.batch_size = batch_sizey

        with open(csv_file, "r") as file:

            self.y = np.zeros((sum(1 for line in file), self.output_size))
            file.seek(0)

            reader = csv.reader(file, delimiter=",")
            for index, (scaled_img_path, _, _, x_com, y_com, z_com, _, _, _, _, _, _) in enumerate(reader):

                if self.output_size == 2:
                    self.y[index][0] = x_com
                    self.y[index][1] = y_com
                    
                self.paths.append(scaled_img_path)

            #print (str(self.paths))


    def __len__(self):
        return math.ceil(len(self.y) / self.batch_size)

    def __getitem__(self, idx):
        
        batch_y = self.y[idx * self.batch_size:(idx + 1) * self.batch_size]
        batch_x = self.paths[idx * self.batch_size:(idx + 1) * self.batch_size]

        images = self.__load_images(batch_x)
        images = preprocess_input(images)

        return images, batch_y

def createModel(size, alpha,output_size):
    
    model = MobileNetV2(input_shape=(size, size, 3), include_top=False, alpha=alpha)

    for layer in model.layers:              #batch normalization layers nie podlegaja treningowi
        if "bn" or "BN" in layer.name :
            layer.trainable = False

    x = model.layers[-1].output
    if size == 96:
        kernel_size_adapt = 3
    elif size == 128:
        kernel_size_adapt = 4
    
    x = Conv2D(output_size, kernel_size=kernel_size_adapt, name="coords")(x)
    x = Reshape((output_size,))(x)

    if output_size== 2:
        rospy.logwarn("CREATED MODEL 2D XY Output")

    return Model(inputs=model.input, outputs=x)

def train(model, epochs, batch_size, patience, threads, train_csv, validation_csv, models_weight_checkpoints_folder, logs_folder, load_weight_starting_file=None, output_size=2, initial_learning_rate=0.0001, min_learning_rate = 1e-8):
    
    train_datagen = DataSequence(train_csv, batch_size, output_size)
    validation_datagen = DataSequence(validation_csv, batch_size,output_size)

    if load_weight_starting_file:
        rospy.logwarn("Preload Weights, to continue prior training...."+str(load_weight_starting_file))
        model.load_weights(load_weight_starting_file)
    else:
        rospy.logerr("Starting from empty weights.......")

    adam_optim = Adam(lr=initial_learning_rate, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
    model.compile(loss="mse", optimizer="adam", metrics=[tf.keras.metrics.MeanSquaredError()])

    full_model_file_name = "model{loss:.6f}.h5"
    model_file_path = os.path.join(models_weight_checkpoints_folder, full_model_file_name)

    checkpoint = ModelCheckpoint(model_file_path, monitor="loss", verbose=1, save_best_only=True,save_weights_only=True, mode="auto", period=1)
    
    stop = EarlyStopping(monitor="val_loss", patience=patience*3, mode="auto")

    reduce_lr = ReduceLROnPlateau(monitor="loss", factor=0.2, patience=patience, min_lr=min_learning_rate, verbose=1, mode="auto")

    tensorboard_clb = TensorBoard(log_dir=logs_folder, histogram_freq=0,write_graph=True, write_images=True)


    K.set_learning_phase(1)

    #model.summary()
                #pobiera batche w rozmiarze 32 , az skoncza sie dane trenujace
    model.fit_generator(generator=train_datagen,epochs=epochs,validation_data=validation_datagen,  
                        callbacks=[checkpoint, reduce_lr, stop, tensorboard_clb],workers=threads,use_multiprocessing=True,
                        shuffle=True,verbose=1)

def readTensorFlowFile():
    package_path = "/home/igor/mobile/src/igorudnicki/"
    path_to_events_file= package_path + "logs_tensor/events.out.tfevents.1614103753.igor"
    for e in tf.train.summary_iterator(path_to_events_file):
        for v in e.summary.value:
            if v.tag == 'loss' or v.tag == 'accuracy':
                print(v.simple_value) 

def createLogsCheckpointsFolders():

    package_path = "/home/igor/mobile/src/igorudnicki/"
    csv_folder_path = package_path + "csv/"
    train_csv_file = csv_folder_path + "training.csv"
    validation_csv_file = csv_folder_path + "validation.csv"

    models_weight_checkpoints_folder = os.path.join(package_path, "model_weight_checkpoints_gen")
    logs_folder = os.path.join(package_path, "logs_tensor")
       
    # We clean up the training folders
    if os.path.exists(models_weight_checkpoints_folder):
        shutil.rmtree(models_weight_checkpoints_folder)
    os.makedirs(models_weight_checkpoints_folder)
    print("Created folder=" + str(models_weight_checkpoints_folder))
    
    if os.path.exists(logs_folder):
        shutil.rmtree(logs_folder)
    os.makedirs(logs_folder)
    print("Created folder=" + str(logs_folder))

    return train_csv_file,validation_csv_file,models_weight_checkpoints_folder,logs_folder 

if __name__ == "__main__":

    rospy.init_node('training_1_node', anonymous=True, log_level=rospy.WARN)

    image_size = 128
    alpha = 1.0
    epochs = 40
    batch_size = 32
    patience = 5
    threads = 2 
    output_size = 2
    initial_learning_rate = 0.0001
    min_learning_rate = 0.00000001
    load_weight_starting_file = None

    model = createModel(image_size, alpha, output_size)

    if load_weight_starting_file:
        rospy.logwarn("Preload Weights "+str(load_weight_starting_file))
        model.load_weights(load_weight_starting_file)
    else:
        rospy.logerr("Starting from empty weights")

    plot_model(model, to_file='./model.png', show_shapes=True)

    load_weight_starting_file = None

    train_csv_file,validation_csv_ofile,models_weight_checkpoints_folder,logs_folder = createLogsCheckpointsFolders()   

    #readTensorFlowFile()

    train(model,epochs,batch_size,patience,threads,train_csv_file,validation_csv_file,
        models_weight_checkpoints_folder,logs_folder,load_weight_starting_file,output_size,initial_learning_rate,min_learning_rate)
