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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped, Vector3
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

from velocity_controller import VelocityController



class Movement(OffboardCommon):
    """
    Run a mission
    """

    def __init__(self):
        super().__init__()
        rospy.init_node('offb')

        #self.rate = rospy.Rate(10)

        self.mission_item_reached = -1  # first mission item is 0
        self.mission_name = ""

        # Pubs
        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

        # Subs
        #self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=self.velocity_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)


        # Services
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)


        # ATTITUDE STUFF
        #self.att = AttitudeTarget()

        #self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        #self.att_thread = Thread(target=self.send_att, args=())
        #self.att_thread.daemon = True
        #self.att_thread.start()


        # Velocity stuff
        self.target = PoseStamped()
        self.radius = 0.2

        self.set_velocity_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1) # Queue_size is 10 as the example from Intel's Aero-drone's sample app
        

        # send setpoints in seperate thread to better prevent failsafe
        self.ctrl_thread = Thread(target=self.send_thread, args=())
        self.ctrl_thread.daemon = True
        self.ctrl_thread.start()



    def send_thread(self):
        rate = rospy.Rate(10)  # Hz
        
        self.target.header = Header()
        self.target.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.target.header.stamp = rospy.Time.now()
            self.set_velocity_pub.publish(self.target)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        #quaternion = quaternion_from_euler(0, 0, yaw) TODO: Find some other or port this example of conversion to py3
        self.target.pose.orientation = Quaternion(0,0,0,1)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.target.pose.position.x,
                                   self.target.pose.position.y,
                                   self.target.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                exit(0)


    # Test method
    #
    def test_velctl(self):
        """Test offboard Position control"""


        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        for i in range(0, 20):
            self.set_velocity_pub.publish(self.target)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        simulation_done = False
        rate = rospy.Rate(2)
        if (self.local_position.pose.position.x > 200):
            rospy.loginfo("We have escaped.")
            simulation_done = True

        try:
            rate.sleep()
        except rospy.ROSException:
            rospy.loginfo("Something bad happened")
            exit(0)

        if simulation_done:
            self.set_mode("AUTO.LAND", 5)
            self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 90, 0)
            self.set_arm(False, 5)
    
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
                     (0, 0, 20))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

        


if __name__ == '__main__':
    try:
        move = Movement()

        #move.test_velctl()
        move.test_posctl()

        #rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)
