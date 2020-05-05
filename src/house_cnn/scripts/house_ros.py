#!/usr/bin/env python3
import os
import sys
import random
import skimage.io

# Root directory of the project
ROOT_DIR = os.path.abspath("./")
PARENT_DIR = os.path.abspath(os.path.join(ROOT_DIR, os.pardir))
INCLUDE_DIR = os.path.join(PARENT_DIR, "include")
LIB_DIR = os.path.join(INCLUDE_DIR, "model")
# Import Mask RCNN
sys.path.append(LIB_DIR)  # To find local version of the library
import mrcnn.model as modellib
from mrcnn import visualize
from mrcnn.config import Config
import argparse

import rospy
from sensor_msgs.msg import Image
# and messages

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


    def segment_image_callback(self, image):
        # Load a random image from the images folder
        #         
        # Run detection
        self.results.append(self.model.detect([image], verbose=1))

    def show(self):

        # Visualize newest results
        r = self.results[len(self.results) - 1][0]
        visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                                    self.class_names, r['scores'])


if __name__ == '__main__':
    try:
        # Create an instance of this class and load the model weights etc. (takes a while, so might as well do it asap)
        house = HouseRCNN()

        # Wait for an image from somewhere?
        image = rospy.wait_for_message('imageasdf', Image)

        # Use the CNN to segment the image we just received
        house.segment_image_callback(image)

        # Visualize the results
        house.show()


    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)

