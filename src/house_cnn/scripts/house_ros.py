#!/usr/bin/env python3
import os
import sys
import random
import skimage.io

# Root directory of the project
CURR_DIR = os.path.dirname(os.path.realpath(__file__)) # os.path.abspath("./")
PARENT_DIR = os.path.abspath(os.path.join(CURR_DIR, os.pardir))
INCLUDE_DIR = os.path.join(PARENT_DIR, "include")
LIB_DIR = os.path.join(INCLUDE_DIR, "model")
# Import Mask RCNN

sys.path.append(LIB_DIR)  # To find local version of the library
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn.config import Config
import argparse

import rospy
from sensor_msgs.msg import Image, CompressedImage
from offb.msg import BuildingPolygonResult, CameraStuff
# and messages

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math


class InferenceConfig(Config):
    # Set batch size to 1 since we'll be running inference on 
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    NAME = "houses"
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    NUM_CLASSES = 1 + 3


class HouseRCNN():
    def __init__(self):
        # INIT NODE
        rospy.init_node('house_cnn')      

        self.impression_sub = rospy.Subscriber('/impression_image_from_offboard', Image, callback=self.append_to_images, queue_size=2)
        #rospy.Subscriber('/camera_and_distances', CameraStuff, queue_size=1)

        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(INCLUDE_DIR, "model/logs")
        # Local path to trained weights file
        HOUSE_MODEL_PATH = os.path.join(INCLUDE_DIR, "house_full_20k_5_5.h5")
        config = InferenceConfig()

        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

        # Load trained weights
        self.model.load_weights(HOUSE_MODEL_PATH, by_name=True)

        # Class names
        self.class_names = ['BG', 'door', 'garage_door', 'window']

        # Empty list of results
        self.results = []
        
        # Empty list of images that gets appended to every time the drone takes an impression image
        self.images = []

        # CV2 bridge to convert images from ros-messages
        self.bridge = CvBridge()

        self.polygon = BuildingPolygonResult()
        self.polygon = rospy.wait_for_message('/house_results_from_overpass_local', BuildingPolygonResult)
        
        self.camera = CameraStuff()
        self.camera = rospy.wait_for_message('/camera_and_distances', CameraStuff)
        

    def append_to_images(self, data):
        rospy.loginfo("Acquired image from Offboard")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.images.append(cv_image)


    def segment_image(self, image, counter):
        # Right before segmentation cut it so that only the facade we are looking at is in the frame.
        # We can calculate the current field of view with the distance that we currently are from the walls and the AOV.etc
        fov = []
        fov.append(2.0 * (np.tan(self.camera.aov[0]*math.pi / 180.0 * 0.5) * self.camera.distances_from_walls[counter]))
        fov.append(2.0 * (np.tan(self.camera.aov[1]*math.pi / 180.0 * 0.5) * self.camera.distances_from_walls[counter]))
        
        # Field of view should now be in meters that spans the entire image, so we find a ratio between that FOV and the length of that wall
        ratio = self.polygon.distances[counter] / fov[0]
        slice_width_in_pixels = ratio * self.camera.resolution[1]

        x = int((self.camera.resolution[1] * 0.5) - slice_width_in_pixels * 0.5) # TODO: THIS IS WRONG
        xx = int((self.camera.resolution[1] * 0.5) + slice_width_in_pixels * 0.5)
        
        y = 0
        yy = int(self.camera.resolution[0])
        
        rospy.loginfo(x)
        rospy.loginfo(xx)
        rospy.loginfo(y)
        rospy.loginfo(yy)

        cropped_image = image[y:yy, x:xx]

        # Run detection
        self.results.append(self.model.detect([image], verbose=1))

    def show(self, image):

        # Visualize newest results
        r = self.results[len(self.results) - 1][0]
        visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                                    self.class_names, r['scores'])


if __name__ == '__main__':
    try:
        # Create an instance of this class and load the model weights etc. (takes a while, so might as well do it asap)
        house = HouseRCNN()
        rate = rospy.Rate(2)
        image_counter = 0
        
        while not rospy.is_shutdown():
            
            if image_counter < len(house.images):
                # Use the CNN to segment the image we just received
                house.segment_image(house.images[image_counter], image_counter)

                # Visualize the results
                house.show(house.images[image_counter])

                # Increment the counter so that we won't get in here before we get a new image
                image_counter += 1

            try:
                rate.sleep()
            except rospy.ROSException:
                rospy.loginfo("Shutting down")
                exit(0)




    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)

