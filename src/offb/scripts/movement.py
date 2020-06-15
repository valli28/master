#!/usr/bin/env python3

#***************************************************************************
#
#   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#

# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.

# I, Valthor, changed it to Python3 as pymavlink claims to work with it now.
# I also tailored this for the purpose of actually flying.

from __future__ import division

import rospy

#from px4tools import ulogf
from geometry_msgs.msg import Pose, Point, Twist, PoseArray
from sensor_msgs.msg import CompressedImage, Image
from pymavlink import mavutil
from offboard import OffboardCommon
from threading import Thread
#from tf.transformations import quaternion_from_euler
from six.moves import xrange
import numpy as np


from velocity_controller import VelocityGuidance

from offb.msg import Bool, BuildingPolygonResult



class Movement(OffboardCommon):
    def __init__(self):
        super().__init__()
        rospy.init_node('offb')

        # Velocity stuff
        self.target = Twist()
        self.target.linear.x = 0
        self.target.linear.y = 0
        self.target.linear.z = 0
        self.target.angular.x = 0
        self.target.angular.y = 0
        self.target.angular.z = 0
        
        self.pos_goal = Pose()

        self.radius = 0.5 # Radius of goal-reached

        self.set_velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1) # Queue_size is 10 as the example from Intel's Aero-drone's sample app

        self.tell_planner_that_drone_is_ready_pub = rospy.Publisher('/gps_ready_from_offboard', Bool)

        self.camera_sub = rospy.Subscriber('/cgo3_camera/image_raw', Image, self.camera_callback, queue_size=1)

        self.image_pub = rospy.Publisher('impression_image_from_offboard', Image)
        

        self.vel_controller = VelocityGuidance(request_velocity=1)
        
        self.ready = False

        # send setpoints in seperate thread to better prevent failsafe
        self.ctrl_thread = Thread(target=self.send_thread, args=())
        self.ctrl_thread.daemon = True
        self.ctrl_thread.start()

    def camera_callback(self, ros_data):
        '''Callback function for image from camera. Is supposedly a "ros image" that needs to be converted with OpenCV '''

        if not self.ready:
            return None

        # Save image in this class...
        self.image = ros_data

    def send_thread(self):
        thread_rate = rospy.Rate(20)  # Hz

        while not rospy.is_shutdown():
            self.set_velocity_pub.publish(self.target)
            #string = "x: " + str(self.target.linear.x) + " y: " + str(self.target.linear.y) + " z: " + str(self.target.linear.z)
            #rospy.loginfo_throttle(2, string)
            try:  # prevent garbage in console output when thread is killed
                thread_rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def go_to_position(self, x, y, z, yaw, speed, rate):
        string = "Going to position " + str([x, y, z])
        rospy.loginfo(string)
        # Horizontal position controller
        self.pos_goal.position.x = x
        self.pos_goal.position.y = y
        self.pos_goal.position.z = z
        self.pos_goal.orientation.z = yaw

        self.vel_controller.set_target(self.pos_goal)
        #rospy.loginfo(self.local_position.pose.position.z)

        while not self.vel_controller.reached(self.radius) == True:
            #rospy.loginfo_throttle(0.5, "Going to position")
            #string = "Position = " + str([self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z])
            #rospy.loginfo_throttle(1, string)
            try:
                rate.sleep()
            except rospy.ROSException:
                exit(0)
            self.target = self.vel_controller.update_point(self.local_position, speed)

    def follow_line(self, A, B, speed, rate):
        string = "Beginning to follow line from A" + str([A.x, A.y, A.z]) + " to B" + str([B.x, B.y, B.z])
        rospy.loginfo(string)
        self.vel_controller.set_line(A, B)

        self.pos_goal.position = B
        
        while not self.vel_controller.reached(self.radius) == True:
            #rospy.loginfo_throttle(0.5, "Following line")
            #string = "Position = " + str([self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z])
            #rospy.loginfo_throttle(1, string)
            try:
                rate.sleep()
            except rospy.ROSException:
                exit(0)
            self.target = self.vel_controller.update_line(self.local_position, speed)

    def loiter(self, rate, duration=2):
        string = "Beginning to loiter for " + str(duration) + " seconds"
        rospy.loginfo(string)

        self.vel_controller.set_target(self.pos_goal)

        for i in xrange(duration * int(1 / rate.sleep_dur.to_sec())):
            #rospy.loginfo_throttle(0.2, "Loitering...")
            self.target = self.vel_controller.update_point(self.local_position, request_velocity=0.1)

            #string = "Position = " + str([self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z])
            #rospy.loginfo_throttle(1, string)

            try:
                rate.sleep()
            except rospy.ROSException:
                exit(0)
            
    def loiter_and_pic(self, rate):
        '''
        Starts to loiter and when we've loitered a bit, we take an image. When the image has been taken, we can continue
        '''
        self.loiter(rate, duration=3)
        # The loiter function does nothing but just loiter. Maybe we should start a stand-alone thread that actually takes the image...

        # Take a picture
        image = self.image

        # Lets just try to see if we can take pic after loitering, and then go straight back to loitering.
        self.loiter(rate, duration=1)

        return image

    def take_off(self, takeoff_altitude, rate):
        rospy.loginfo("Taking off")
        string = "Ascending to loiter position of " + str(takeoff_altitude) + " m"
        rospy.loginfo(string)
        self.go_to_position(self.home_position.position.x, self.home_position.position.y, takeoff_altitude, 0, speed=1.0, rate=rate)
        string = "Takeoff altitude of " + str(takeoff_altitude) + " m reached"
        rospy.loginfo(string)


if __name__ == '__main__':
    try:
        move = Movement()
        # make sure the simulation is ready to start the mission
        move.wait_for_topics(60)
        move.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        move.log_topic_vars()

        move.set_mode("OFFBOARD", 5)

        # Now that we are in offboard, we actually ready to go, but before we arm the drone, we want to perform some analysis first
        # and that is to check for surrounding buildings, and we want to analyse the one that is closest to us. 
        move.tell_planner_that_drone_is_ready_pub.publish(True)
        move.ready = True
        # And now, in turn, we wait until the planner gives us something to work with.
        poses = PoseArray()
        poses = rospy.wait_for_message('/impression_poses_from_planner', PoseArray)
        
        move.set_arm(True, 5)

        rospy.loginfo("Run mission")

        mission_done = False
        impressions_done = False
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            move.take_off(takeoff_altitude = 2, rate=rate)
            move.loiter(rate, duration=2)
            images = []
            # Fly the impression poses
            for i in range(0, len(poses.poses), 2): # For loop that counts by two every loop.
                move.go_to_position(poses.poses[i].position.x, poses.poses[i].position.y, poses.poses[i].position.z, poses.poses[i].orientation.z, speed=4, rate=rate)
                image = []
                if poses.header.frame_id == "ci":
                    move.loiter(rate) #loiter and dont take picture
                    move.go_to_position(poses.poses[i+1].position.x, poses.poses[i+1].position.y, poses.poses[i+1].position.z, poses.poses[i+1].orientation.z, speed=4, rate=rate) #go to impression pose
                    image = move.loiter_and_pic(rate)#loiter and take picture
                    images.append(image) 

                elif poses.header.frame_id == "ic":
                    move.loiter_and_pic(rate) #loiter and take picture
                    move.go_to_position(poses.poses[i+1].position.x, poses.poses[i+1].position.y, poses.poses[i+1].position.z, poses.poses[i+1].orientation.z, speed=4, rate=rate) #go to impression pose
                    image = move.loiter(rate) #loiter and dont take picture
                    images.append(image) 

                move.image_pub.publish(image)
                        
            facade_poses = []
            impressions_done = True
            for i in range(0, len(facade_poses)):
                # Interpolate between facade poses
                print("hey")

            mission_done = True
            try:
                rate.sleep()
            except rospy.ROSException:
                rospy.loginfo("Shutting down")
                exit(0)

            if mission_done:
                rospy.loginfo("Simulation done. Exiting")
                move.set_mode("AUTO.LAND", 5)
                move.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 90, 0)
                move.set_arm(False, 5)

                break
            

    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)
