#!/usr/bin/env python3
from pysolar.solar import *
import datetime
import numpy as np
import utm
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import cm


#Import custom classes
from sun import Sun
from boundary import Boundary
from mission import Mission
from house_polygon import PolygonExtractor

from threading import Thread
import rospy

from offb.msg import Bool, BuildingPolygonResult
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import Pose, PoseArray

import math

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

        self.impression_keyframes_pub = rospy.Publisher('/impression_poses_from_planner', PoseArray, queue_size=5) # Poses to take one image of each facade of the house

        self.inspection_keyframes_pub = rospy.Publisher('/keyframes_poses_from_planner', PoseArray, queue_size=5)
        #coordinates = np.array([55.3729781, 10.4008157])

    def impression(self, drone_lat, drone_lon):

        lat = drone_lat #55.471689
        lon = drone_lon #10.325267

        # Convert to UTM
        utm_center = utm.from_latlon(lat, lon)

        self.s = Sun(lat, lon)
        self.m = Mission(0.8, 0.8)

        local1 = [utm_center[0] - utm1[0], utm_center[1] - utm1[1]]
        local2 = [utm_center[0] - utm2[0], utm_center[1] - utm2[1]]
        local3 = [utm_center[0] - utm3[0], utm_center[1] - utm3[1]]
        local4 = [utm_center[0] - utm4[0], utm_center[1] - utm4[1]]

        height = 2.0
        east_wall = Boundary(local1[0], local1[1], 0.0, local2[0], local2[1], 0.0, local1[0], local1[1], height, "wall", self.s.get_lightsource())
        south_window = Boundary(0.3857, -2.1862, 0.5, 1.3943, -1.532, 0.8, 0.3857, -2.1862, 0.5+0.6, "window", self.s.get_lightsource())
        north_wall = Boundary(local2[0], local2[1], 0.0, local3[0], local3[1], 0.0, local2[0], local2[1], height, "wall", self.s.get_lightsource())
        fake_east_window = Boundary(2.292, 0.424, 0.8, 1.699, 1.342, 0.8, 2.292, 0.424, 0.8+0.5, "window", self.s.get_lightsource())
        west_wall = Boundary(local3[0], local3[1], 0.0, local4[0], local4[1], 0.0, local3[0], local3[1], height, "wall", self.s.get_lightsource())
        fake_west_window = Boundary(-2.4586, -1.0349, 0.8, -1.9872, -2.0232, 0.8, -2.4586, -1.0349, 0.8+0.5, "window", self.s.get_lightsource)
        south_wall = Boundary(local4[0], local4[1], 0.0, local1[0], local1[1], 0.0, local4[0], local4[1], height, "wall", self.s.get_lightsource())

        #ground_plane = Boundary(-5, -5, 0, 5, -5, 0, -5, 5, 0, "grass", s.get_lightsource())

        self.list_of_reflective_boundaries = [fake_east_window, south_window, fake_west_window]
        list_of_walls = [east_wall, north_wall, west_wall, south_wall]


        self.fig = plt.figure(num=1, clear=True)
        self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')


        east_wall.plot(self.ax)
        south_window.plot(self.ax)
        north_wall.plot(self.ax)
        fake_east_window.plot(self.ax)
        west_wall.plot(self.ax)
        fake_west_window.plot(self.ax)
        south_wall.plot(self.ax)

        #ground_plane.plot(ax)

        self.s.cast_on(self.list_of_reflective_boundaries, self.ax)
        self.m.draw_mission_planes(list_of_walls, self.ax)


        self.ax.set_xlabel('X: East')
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylabel('Y: North')
        self.ax.set_ylim(-10, 10)
        self.ax.set_zlabel('Z: Up')
        self.ax.set_zlim(-10, 10)


        axnext = plt.axes([0.85, 0.25, 0.10, 0.075])
        self.bnext = Button(axnext, 'Path')


        slider_box = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
        self.time_slider = Slider(slider_box, 'UNIX Time', valmin=self.s.date.timestamp() - 60.0*60.0*12.0, valmax=self.s.date.timestamp() + 12.0*60.0*60.0, valinit=self.s.date.timestamp())

    def update(self, val):
        self.s.date = datetime.datetime.fromtimestamp(self.time_slider.val, tz=pytz.timezone("Europe/Copenhagen"))
        self.s.cast_on(self.list_of_reflective_boundaries, self.ax)
        self.fig.canvas.draw_idle()
        self.m.check_for_reflection(self.s, self.ax)


    def button_callback(self, event):
        self.m.generate_mission(self.ax)

    def generate_impression_poses(self, house, closest_node, wall_lenghts, building_height, local_coords):
        # and remember that the nodes in the house variable and the distances are ordered correctly corresponding to the order which they are connected.
        rospy.loginfo("I'm in the generate impression poses function now")

        building = BuildingPolygonResult()
        ''' BuildingPolygonResult.msg
        bool found_building
        string building_name
        string building_type
        float64[] distances
        '''
        building.found_building = True
        building.building_name = house.tags.get("name", "n/a")
        building.building_type = house.tags.get("building", "n/a")
        building.distances = wall_lenghts

        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = "impression_poses"

        mission = Mission(80, 75)

        poses_list = []
        for i in range(len(wall_lenghts)):
            pose = Pose()
            distance_from_wall = mission.calculate_distance_from_wall(wall_lenghts[i])

            # Now we have everything we need to generate the point at which the drone has to be to look at the wall.
            # We take the coordinates of the endpoints of the walls
            point_a = local_coords[i]
            point_b = local_coords[i+1]

            # TODO: How do we make sure that we don't make a point inside the building?
            # It seems that the nodes are always defined clockwise, which means that if we draw the vector from a to b, the impression_point should be to the "left"
            # if we are looking in the same direction as the a-b vector

            vector_ab = point_b - point_a

            # We half that vector since we are doing that kind of triangle-calculation
            vector_ab *= 0.5

            # We rotate this vector anti-clockwise (so by a positive rotation-angle theta)
            # The angle is 180-fov divided by two
            theta = (180 - mission.fov[0]) *0.5
            theta = math.radians(theta)

            # We use the rotation-matrix to rotate the vector 
            vector_ac = np.array([math.cos(theta)* vector_ab[0] - math.sin(theta)*vector_ab[1], math.sin(theta)*vector_ab[0] + math.cos(theta)*vector_ab[1]])
            
            # This vector is sadly not the correct size since it is as long as vector_ab. 
            # We have to enlarge it by multiplying it with a scalar corresponding to the ratio between the sides of the triangle and half the wall
            # The length of the arms on the triangle is calculated by its baseline and height, which we know
            a = 0.5*math.sqrt(wall_lenghts[i]**2 + 4*distance_from_wall**2)

            # Now we have what is supposed to be how long the vector is. 
            vector_ac *= a / (wall_lenghts[i]*0.5)

            # If we add vector_ac to point a, we now have point c, which is the impression position.
            impression_position_xy = point_a + vector_ac
            # Generate a position that is halfway between the two nodes, as well as a distance away form the facade (like a triangle)
            pose.position.x = impression_position_xy[0]
            pose.position.y = impression_position_xy[1]
            pose.position.z = int(building_height) / 2 #TODO The height must be something based on the angle of the camera
            
            theta = math.radians(-90)
            heading = np.array([math.cos(theta)* vector_ab[0] - math.sin(theta)*vector_ab[1], math.sin(theta)*vector_ab[0] + math.cos(theta)*vector_ab[1]])
            heading = math.atan(heading[1]/heading[0])
            pose.orientation.z = heading
            
            string = "Point " + str(i) + " xy: (" + str(pose.position.x) + ", " + str(pose.position.y) + str(")")
            rospy.loginfo(string)
            poses_list.append(pose)

        poses.poses = poses_list

        return building, poses

    def generate_keyframes_for_inspection(self, segmented_image):
        # Something someting
        rospy.loginfo("I'm in the generate keyframes for inspection function now")
        poses = PoseArray()
        return poses


    def something(self):
        print("hey")
        self.time_slider.on_changed(self.update)

        self.bnext.on_clicked(self.button_callback)

        plt.show()

if __name__ == '__main__':
    try:
        # First, we initialize the planner which in turn gets to know where we are in the world.
        planner = Planner(building_height = 3) # The estimated building height is measured by our pilot. Can maybe be extracted from some height-map from some database but... i mean...
        # The init-function calls the "wait for message from offboard" which puts this node in a loop while it waits for a GPS coordinate of the drone (GPSFIX)

        polygon = PolygonExtractor(planner.coordinates, radius=20) # Radius is in meters
        closest_house, closest_node, distances_between_nodes, building_height, local_coords = polygon.get_closest_building()
        # Now that we have the collection of nodes, the closest node and the distance between all the nodes, let us run the planner accordingly.

        # For the final product, a google maps widget with the coordinates that we just fetched would confirm with the pilot that we are indeed at the correct site and building.
        # But for this proof of concept, we just run it.

        building_info, impression_poses = planner.generate_impression_poses(closest_house, closest_node, distances_between_nodes, building_height, local_coords)
        #Publish the poses to make the impressions.
        planner.impression_keyframes_pub.publish(impression_poses)
        planner.house_result_pub.publish(building_info)

        # and now get all the impression images as they come and "map" them over to a tilemap with regions to avoid etc.
        #impression_image_from_camera = rospy.wait_for_message('impression_image_from_camera', Image) # THIS IS SUPPOSED TO BE IN THE SEGMENTATION NODE
        # publish this unsegmented impression image to the semantic segmentation node
        segmented_image = rospy.wait_for_message('/segmented_impression_image', Image)
        


        inspection_keyframes = planner.generate_keyframes_for_inspection(segmented_image)

        planner.inspection_keyframes_pub.publish(inspection_keyframes)

    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)




