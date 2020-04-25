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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped, Quaternion, Vector3
from mavros import mavlink
from mavros_msgs.msg import State, ExtendedState, Mavlink, WaypointReached, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil
from offb import OffboardCommon
from threading import Thread
from tf.transformations import quaternion_from_euler
from six.moves import xrange
from std_msgs.msg import Header

from velocity_controller import VelocityController



class Movement(OffboardCommon):
    """
    Run a mission
    """

    def __init__(self):
        super().__init__()
        rospy.init_node('offb')

        self.rate = rospy.Rate(10)

        self.mission_item_reached = -1  # first mission item is 0
        self.mission_name = ""

        # Pubs
        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.set_velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) # Queue_size is 10 as the example from Intel's Aero-drone's sample app

        # Subs
        #self.velocity_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=self.velocity_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)


        # Services
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)


        # ATTITUDE STUFF
        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()



        self.current_velocity = TwistStamped()
        self.desired_velocity = TwistStamped()

        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()

        self.isReadyToFly = False


    #
    # Helper methods
    #

    
    def distance_to_wp(self, lat, lon, alt):
        """alt(amsl): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(alt - self.altitude.amsl)

        rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
        return d, alt_d

    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion(*quaternion_from_euler(-0.25, 0.15,
                                                                 0))
        self.att.thrust = 0.7
        self.att.type_mask = 7  # ignore body rate

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method
    #
    def test_attctl(self):
        """Test offboard attitude control"""
        # boundary to cross
        boundary_x = 200
        boundary_y = 100
        boundary_z = 20

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
                      format(boundary_x, boundary_y, boundary_z))
        # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x > boundary_x and
                    self.local_position.pose.position.y > boundary_y and
                    self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException:
                exit(0)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   90, 0)
        self.set_arm(False, 5)

    def test_velocity(self):
        # make sure the simulation is ready to start the mission

        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        #self.wait_for_mav_type(10)



        vController = VelocityController()
        vController.setTarget(target)
        self.desired_velocity = vController.update(self.current_pose)

        self.set_mode('OFFBOARD', 10)
        self.set_arm(True, 5)

        while not rospy.is_shutdown():
            if self.isReadyToFly:
                self.desired_velocity = vController.update(self.current_pose)
                print(self.desired_velocity)
                self.set_velocity_pub.publish(self.desired_velocity)
            self.rate.sleep()
        


target = Pose()
target.position.x = 9
target.position.y = 2
target.position.z = 3

if __name__ == '__main__':
    try:
        move = Movement()
        
        #move.test_velocity()
        move.test_attctl()

        #rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)
