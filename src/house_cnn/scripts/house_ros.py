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
from offb.msg import BuildingPolygonResult, CameraStuff, ImageAndRois, Roi
# and messages

import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import subprocess

#from building_ros import HouseCSAIL


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

        # Send the results form the segmentation as a custom message containing the image mat and windows as rois
        self.image_pub = rospy.Publisher('/segmented_image_from_cnn', ImageAndRois)

        self.init_model()

        # Class names
        self.class_names = ['BG', 'door', 'garage_door', 'window']

        # Empty list of results
        self.results = []
        
        # Empty list of images that gets appended to every time the drone takes an impression image
        self.images = []
        self.image_shape = 0

        # CV2 bridge to convert images from ros-messages
        self.bridge = CvBridge()

        self.polygon = BuildingPolygonResult()
        self.polygon = rospy.wait_for_message('/house_results_from_overpass_local', BuildingPolygonResult)
        
        self.camera = CameraStuff()
        self.camera = rospy.wait_for_message('/camera_and_distances', CameraStuff)
        
    def init_model(self):
        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(INCLUDE_DIR, "model/logs")
        # Local path to trained weights file
        HOUSE_MODEL_PATH = os.path.join(INCLUDE_DIR, "house_full_20k_5_5.h5")
        self.config = InferenceConfig()

        # Create model object in inference mode.
        
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=self.config)

        # Load trained weights
        self.model.load_weights(HOUSE_MODEL_PATH, by_name=True)

    def append_to_images(self, data):
        rospy.loginfo("Acquired image from Offboard")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        np_image = np.asarray(cv_image)
        
        self.images.append(np_image)


    def segment_image(self, image, counter):
        
        # We perform cropping and segmenting to try and make a plane that is mostly just facade and then we can put the windows and doors on it.
        
        # Cropping #########################################################################################
        # Right before segmentation cut it so that only the facade we are looking at is in the frame.
        # We can calculate the current field of view with the distance that we currently are from the walls and the AOV.etc
        fov = []
        fov.append(2.0 * (np.tan(self.camera.aov[0]*math.pi / 180.0 * 0.5) * self.camera.distances_from_walls[counter]))
        fov.append(2.0 * (np.tan(self.camera.aov[1]*math.pi / 180.0 * 0.5) * self.camera.distances_from_walls[counter]))
        
        # Field of view should now be in meters that spans the entire image, so we find a ratio between that FOV and the length of that wall
        ratio = self.polygon.distances[counter] / fov[0]
        if ratio > 1.0:
            ratio = 1.0
        rospy.loginfo(self.polygon.distances[counter])
        rospy.loginfo(fov[0])
        rospy.loginfo(ratio)
        slice_width_in_pixels = self.camera.resolution[0] * ratio

        x = int((self.camera.resolution[0] * 0.5) - slice_width_in_pixels * 0.5) # TODO: THIS IS WRONG
        xx = int((self.camera.resolution[0] * 0.5) + slice_width_in_pixels * 0.5)
        
        y = 0
        yy = int(self.camera.resolution[1])
                
        cropped_image = image[y:yy, x:xx]
        #rospy.loginfo(np.shape(cropped_image))

        if self.image_shape != np.shape(cropped_image) and self.image_shape != 0:
            rospy.loginfo("Re-initializing cnn model due to change in image shape")
            del self.model
            del self.config
            self.init_model()
        
        self.image_shape = np.shape(cropped_image)    
           

        # Semantic segmentation ##############################################################################
        dir_path = os.path.dirname(os.path.realpath(__file__))
        par_path = os.path.abspath(os.path.join(dir_path, os.pardir))
        # We have to run the CSAIL segmentation in a seperate shell because it requires the conda environment to be turned on, which our ROS system does not like. 
        # First we need to save an image to the folder where the script is
        
        W = int(self.camera.resolution[0]/4)
        height, width, depth = image.shape
        imgScale = W/width
        newX,newY = image.shape[1]*imgScale, image.shape[0]*imgScale
        scaled_image = cv2.resize(image,(int(newX),int(newY)))

        im_path = par_path + '/semantic-segmentation-pytorch/facade.jpg'
        im_path2 = par_path + '/semantic-segmentation-pytorch/facade2.jpg'
        print(im_path)
        cv2.imwrite(im_path, cv2.cvtColor(scaled_image, cv2.COLOR_RGB2BGR))
        cv2.imwrite(im_path2, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        # Call the .sh script 
        subprocess.call(['./segment_image.sh'], cwd=dir_path, shell=True)

        # Now we wait until we can read a file called "bit_output.png"
        bitmap = []
        while True:
            try:
                bitmap = cv2.imread(par_path + '/semantic-segmentation-pytorch/bit_output.png')
                
                break
            except: 
                rate.sleep()

        bitmap = cv2.bitwise_not(bitmap)
        kernel = np.ones((5,5),np.uint8)
        #bitmap = cv2.erode(bitmap, kernel, iterations=5)
        #bitmap = cv2.dilate(bitmap, kernel, iterations=5)
        bitmap = cv2.morphologyEx(bitmap, cv2.MORPH_OPEN, kernel, iterations=7)

        # Enlarge image back to original size
        W = int(self.camera.resolution[0])
        height, width, depth = bitmap.shape
        imgScale = W/width
        newX,newY = bitmap.shape[1]*imgScale, bitmap.shape[0]*imgScale
        bitmap = cv2.resize(bitmap,(int(newX),int(newY)))
        #print(bitmap.shape)
        #cv2.imshow("Enlarged bitmap", bitmap)

        # crop bitmap to same size as the FoV bounding box
        bitmap = bitmap[y:yy, x:xx]

        # Find bounding box of bitmap
        cols = np.any(bitmap, axis=0)
        rows = np.any(bitmap, axis=1)
        cmin, cmax = np.where(cols)[0][[0, -1]] # X-axis
        rmin, rmax = np.where(rows)[0][[0, -1]] # Y-axis

        cmin += x
        cmax += x
        rmin += y
        rmax += y

        # Finding the minimum of both the crop-method and the segmentation method. This way, we get "best of both worlds"
        bbox = np.array([cmin, cmax, rmin, rmax])

        #print(bbox)
        #cv2.imshow("bitmap test", bitmap)
        #cv2.waitKey(0)

        # Run detection on the bounding box
        image = image[bbox[2]:bbox[3], bbox[0]:bbox[1]]
        self.results.append(self.model.detect([image], verbose=1))

        # Paint "plane" on the image
        #cv2.rectangle(image,(bbox[0],bbox[2]),(bbox[1],bbox[3]),(255,0,0), 3)

        # Visualize newest results
        r = self.results[counter][0]
        #rospy.loginfo(r['masks'])
        visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], self.class_names, r['scores'])

        # The last thing is removal of image from folder
        os.remove(par_path + '/semantic-segmentation-pytorch/bit_output.png') # Remove the file so the next time we while->try, have to wait until the segmentation script is done.
        #cv2.destroyWindow('Enlarged bitmap') 
        return image, r['rois']

    def send_image_and_windows(self, img, rois):
        sent_image = Image()
        sent_image = self.bridge.cv2_to_imgmsg(img, "rgb8")

        rois_msg_array = []
        for i in range(len(rois)):
            #rospy.loginfo(i)
            #rospy.loginfo(rois[i])
            rois_msg_array.append(Roi(rois[i]))

        
        msg = ImageAndRois()
        msg.img = sent_image
        msg.rois = rois_msg_array

        self.image_pub.publish(msg)


if __name__ == '__main__':
    try:
        # Create an instance of this class and load the model weights etc. (takes a while, so might as well do it asap)
        house = HouseRCNN()
        rate = rospy.Rate(2)
        image_counter = 0
        
        while not rospy.is_shutdown():
            
            if image_counter < len(house.images):
                # Use the CNN to segment the image we just received
                image, rois = house.segment_image(house.images[image_counter], image_counter)

                # Send the windows and the image of the facade.
                house.send_image_and_windows(image, rois)

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

