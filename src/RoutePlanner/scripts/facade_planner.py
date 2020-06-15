#!/usr/bin/env python3
from pysolar.solar import *
import datetime
import numpy as np
import math
import utm
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import cm


#Import custom classes
from sun import Sun
from boundary import Boundary

from house_polygon import PolygonExtractor
from mission import Mission

from threading import Thread
import rospy

from offb.msg import BuildingPolygonResult, CameraStuff, Bool, ImageAndRois
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Pose, PoseArray

import copy


# This class might actually be split up into two classe since it actually serves two different purposes that are being performed at different times.
class Planner():
    def __init__(self, building_height):
        rospy.init_node('offb')
        self.height = building_height
        self.GPS_ready = False

        # Wait for Offboard and drone to be ready
        ready = rospy.wait_for_message('/gps_ready_from_offboard', Bool, timeout=rospy.Duration(60*5)) # Wait for offboard to get GPS fix for 5 minutes, because sometimes GPS locks take long to get when a board is booted up the first time

        if ready:
            nav = NavSatFix()
            # And now put the node into a waiting loop to wait for GPS Lock from Offboard
            nav = rospy.wait_for_message('mavros/global_position/global', NavSatFix)
            self.coordinates = np.array([nav.latitude, nav.longitude])
        else:
            rospy.loginfo("Never got position confirmation from Offboard")

        self.house_result_pub = rospy.Publisher('/house_results_from_overpass_local', BuildingPolygonResult, queue_size=5) # Information about the house
        self.camera_stuff_pub = rospy.Publisher('/camera_and_distances', CameraStuff, queue_size=1)
        self.impression_keyframes_pub = rospy.Publisher('/impression_poses_from_planner', PoseArray, queue_size=5) # Poses to take one image of each facade of the house
        self.inspection_keyframes_pub = rospy.Publisher('/keyframes_poses_from_planner', PoseArray, queue_size=5)

        #self.segmented_image_sub = rospy.Subscriber('segmented_image_from_cnn', cnnResult, self.segmented_image_callback, queue_size=5)

        self.image_and_rois_sub = rospy.Subscriber('/segmented_image_from_cnn', ImageAndRois, callback=self.image_and_rois_callback)

        self.get_tiles_ready = False

        self.images_list = []
        self.rois_list = []

        self.s = Sun(self.coordinates[0], self.coordinates[1])
        self.m = Mission(80, 80)
        self.model_walls = []
        self.model_windows = []

        self.fig_thread = Thread(target=self.draw_thread, args=())
        self.fig_thread.daemon = True

        self.fig = []
        self.ax = []
        

    def image_and_rois_callback(self, data):
        image = data.img
        rois = data.rois

        self.images_list.append(image)
        self.rois_list.append(rois)

        if self.get_tiles_ready == False:
            
            self.get_tiles_ready = True


    def send_camera_and_mission_information_to_cnn(self, mission, distances):
        ''' Format of message is
        float64[] aov
        float64 focal_length
        float64 resolution
        float64[] distances_from_walls
        '''
        c = CameraStuff()
        c.aov = [mission.aov[0], mission.aov[1]]
        c.focal_length = mission.f
        c.resolution =  [mission.sensor_resolution[0], mission.sensor_resolution[1]]
        c.distances_from_walls = distances

        self.camera_stuff_pub.publish(c)

    def build_walls(self, polygon_result, local_coords):
        rospy.loginfo("Planner recieved information about the house. Building walls for detailed model")
        #model_walls = []
        for i in range(len(local_coords) - 1):
            # Use the local coordinates to build walls
            self.model_walls.append(Boundary(local_coords[i][0], local_coords[i][1], 0.0, local_coords[i+1][0], local_coords[i+1][1], 0.0, local_coords[i][0], local_coords[i][1], polygon_result.building_height, "wall", self.s.get_lightsource()))


        # Let's also initiate the figure and plot all the walls
        self.fig = plt.figure(num=1, clear=True)
        self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')

        # For loop that goes through all the model walls and plots them
        for wall in self.model_walls:
            wall.plot(self.ax)

        #ground_plane.plot(ax)
        self.ax.set_xlabel('X: East')
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylabel('Y: North')
        self.ax.set_ylim(-50, 50)
        self.ax.set_zlabel('Z: Up')
        self.ax.set_zlim(-20, 20)

        axsolarbutton = plt.axes([0.85, 0.25, 0.10, 0.075])
        self.solar_button = Button(axsolarbutton, 'Solar Check')


        self.m.draw_mission_planes(self.model_walls, self.ax)

        axbrushfirebutton = plt.axes([0.85, 0.10, 0.10, 0.075])
        self.brushfirebutton = Button(axbrushfirebutton, 'Generate mission')


    def put_up_windows(self, facade_image, windows, wall_length, model_wall):
        rospy.loginfo("Planner received information about the windows. Putting windows on walls")
        # Generate an array that takes the time from now and the next 30 minutes, which we will say is the estimated mission duration
        # We assume that the image itself is the plane, which is harsh to assume, but it's the best we've got... 
        # Based on the size of the image (width, height), we make translation factors for x and y to be able to place the windows on the planes.

        # Let's then generate windows in the form that our mission class can understand, taking the bounding boxes of the instance segmentation, aka the rois and make windows out of them
        for i in range(len(windows)):
            # Remember that the coordinates of the rois (windows) are upside down on the y-axis
            # The rois are in the following form : y1,x1, y2,x2
            
            n_xy_tiles = len(model_wall.x)
            n_z_tiles = len(model_wall.z) # this is the y coordinate of the image, but the z coordinate in the world-frame
            
            x1 = n_xy_tiles - round((windows[i].roi[1] / facade_image.width) * n_xy_tiles) - 1 
            y1 = n_z_tiles - round((windows[i].roi[0] / facade_image.height) * n_z_tiles) - 1
            x2 = n_xy_tiles - round((windows[i].roi[3] / facade_image.width) * n_xy_tiles) - 1
            y2 = n_z_tiles - round((windows[i].roi[2] / facade_image.height) * n_z_tiles) - 1

            #rospy.loginfo([x1, y1, x2, y2])
            # We are going to find indecies for the plane and just select the ones that the window covers

            #  TODO: COrrect this method. It's not quite right...

            p1 = np.array([model_wall.x[x1], model_wall.y[x1], model_wall.z[y2]])
            p2 = np.array([model_wall.x[x2], model_wall.y[x2], model_wall.z[y2]])
            p3 = np.array([model_wall.x[x1], model_wall.y[x1], model_wall.z[y1]])

            window = Boundary(p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], p3[0], p3[2], p3[2], "window", self.s.get_lightsource())

            window.plot(self.ax)

            self.model_windows.append(window)

        rospy.loginfo(self.s.date.timestamp())

        return PoseArray()
   

    def generate_keyframes_for_inspection(self, segmented_image):
        # Something someting
        rospy.loginfo("I'm in the generate keyframes for inspection function now")
        return_poses = PoseArray()
        return return_poses

    def solar_button_callback(self, event):
        rospy.loginfo("Analyzing mission planes for solar reflections for the next 30 minutes")
        time_range = range(int(self.s.date.timestamp()), int(self.s.date.timestamp() + 30.0*60.0), 120) # generate a time-instance every 30th second? 30 minutes

        for i in range(len(time_range)):
            self.s.date = datetime.datetime.fromtimestamp(time_range[i], tz=pytz.timezone("Europe/Copenhagen"))
            self.s.cast_on(self.model_windows, self.ax)
            self.fig.canvas.draw_idle()
            self.m.check_for_reflection(self.s, self.ax)
        rospy.loginfo("Done analyzing mission planes for solar reflections for the next 30 minutes")

    def brushfirebutton_callback(self, event):
        self.m.generate_mission(self.ax)

    def draw_thread(self):
        thread_rate = rospy.Rate(20)  # Hz
        plt.show()

        while not rospy.is_shutdown():
            #self.fig.canvas.draw_idle()
            
            

            try:  # prevent garbage in console output when thread is killed
                thread_rate.sleep()
            except rospy.ROSInterruptException:
                pass
        

if __name__ == '__main__':
    try:
        # First, we initialize the planner which in turn gets to know where we are in the world.
        planner = Planner(building_height = 3) # The estimated building height is measured by our pilot. Can maybe be extracted from some height-map from some database but... i mean...
        # The init-function calls the "wait for message from offboard" which puts this node in a loop while it waits for a GPS coordinate of the drone (GPSFIX)
        origin = np.array([55.396142, 10.388953]) # This is the same as PX4_HOME_ LAT and LON
        polygon = PolygonExtractor(planner.coordinates, origin, radius=20) # Radius is in meters
        closest_house, closest_node, building_height, local_coords = polygon.get_closest_building()
        # Now that we have the collection of nodes, the closest node and the distance between all the nodes, let us run the planner accordingly.

        # For the final product, a google maps widget with the coordinates that we just fetched would confirm with the pilot that we are indeed at the correct site and building.
        # But for this proof of concept, we just run it.

        # Let's draw the building in 3D like planes
        # First, build the plane-model walls
        planner.build_walls(polygon, local_coords)
        # Start the thread that updates the matplotlib figure with the mathematical model
        planner.fig_thread.start()

        #building_info, impression_poses = planner.generate_impression_poses(closest_house, closest_node, distances_between_nodes, building_height, local_coords)
        
        building_info, impression_poses = polygon.generate_impression_poses(planner.m)
        polygon.draw()

        rate = rospy.Rate(2)

        # init a counter that keeps track of which facade we are currently working on once we are done finding the overpass stuff
        image_counter = 0
        while not rospy.is_shutdown(): # put the publishes into a loop when we are done drawing to make sure that noone misses it
            planner.solar_button.on_clicked(planner.solar_button_callback)
            planner.brushfirebutton.on_clicked(planner.brushfirebutton_callback)

            if planner.get_tiles_ready != True: # The first state of this loop is doing impressions
                planner.impression_keyframes_pub.publish(impression_poses)
                planner.house_result_pub.publish(building_info)
                planner.send_camera_and_mission_information_to_cnn(planner.m, polygon.wall_distances)
            else: # And the second state is doing detailed positions for the "model" or whatever
                if image_counter < len(planner.images_list):
                    
                    # generate paths using the tile-approach
                    poses = planner.put_up_windows(planner.images_list[image_counter], planner.rois_list[image_counter], polygon.wall_distances[image_counter], planner.model_walls[image_counter])
                    planner.inspection_keyframes_pub.publish(poses)
                    image_counter += 1



            try:
                rate.sleep()
            except rospy.ROSException:
                exit(0)
        
        # TODO: Receive stuff from house_ros.py and mix it with ours and generate the route
        # Do the same kind of approach with a counter each time we get into the callback and the while-loop


        #inspection_keyframes = planner.generate_keyframes_for_inspection(segmented_image)
        #planner.inspection_keyframes_pub.publish(inspection_keyframes)

    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)




