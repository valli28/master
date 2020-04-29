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

PKG = 'px4'

import rospy


import math
import os
#from px4tools import ulog
import sys
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped, Twist, Vector3
from mavros import mavlink
from mavros_msgs.msg import State, ExtendedState, Mavlink, WaypointReached, AttitudeTarget, PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil
from offb import OffboardCommon
from threading import Thread
#from tf.transformations import quaternion_from_euler
from six.moves import xrange
from std_msgs.msg import Header
import numpy as np

from velocity_controller import VelocityController, VelocityGuidance


class Movement(OffboardCommon):
    """
    Run a mission
    """

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
        

        self.radius = 0.5 # Radius of goal-reached

        self.set_velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1) # Queue_size is 10 as the example from Intel's Aero-drone's sample app
        
        #self.vel_controller = VelocityController()
        self.vel_controller = VelocityGuidance(request_velocity=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.ctrl_thread = Thread(target=self.send_thread, args=())
        self.ctrl_thread.daemon = True
        self.ctrl_thread.start()



    def send_thread(self):
        rate = rospy.Rate(50)  # Hz

        while not rospy.is_shutdown():
            self.set_velocity_pub.publish(self.target)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def go_to_position(self, x, y, z):
        # Horizontal position controller
        temp_target = Pose()
        temp_target.position.x = x
        temp_target.position.y = y
        temp_target.position.z = z
        

        self.vel_controller.set_target(temp_target)
        #rospy.loginfo(self.local_position.pose.position.z)

        #while not (z - self.radius) < self.local_position.pose.position.z < (z + self.radius):
        self.target = self.vel_controller.update_point(self.local_position)


    def take_off(self, takeoff_altitude):
        string = "Ascending to loiter position of " + str(takeoff_altitude) + " m"
        #rospy.loginfo(string)
        self.go_to_position(self.home_position.position.x, self.home_position.position.y, takeoff_altitude)
        #string = "Takeoff altitude of " + str(takeoff_altitude) + " m reached"
        rospy.loginfo(string)


        
if __name__ == '__main__':
    try:
        move = Movement()
        # make sure the simulation is ready to start the mission
        move.wait_for_topics(60)
        #for i in range(0, 20):
        #    self.set_velocity_pub.publish(self.target)
        move.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        move.log_topic_vars()
        move.set_mode("OFFBOARD", 5)
        move.set_arm(True, 5)

        rospy.loginfo("run mission")

        mission_done = False
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            rospy.loginfo("Taking off")
            move.take_off(takeoff_altitude = 2)
            
            
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
