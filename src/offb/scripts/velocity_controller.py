# Thanks to darknight-007 and his docking repos on GitHub

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3, Twist, TwistStamped
import math
import numpy as np
from std_msgs.msg import Header
from PID import PID
from squaternion import Quaternion


class VelocityGuidance:
    #output = Twist()

    def __init__(self, request_velocity = 1.0):
        self.point_A = np.array([])
        self.point_B = np.array([])

        self.request_velocity = request_velocity


        self.state = np.array([0, 0, 0])
        self.target = np.array([]) # State and target cannot be initialized to the same thing, or else the reached() check returns true prematurely

        self.Yaw = PID(Kp=0.005, Kd=0.02, maxOut=1.5) # TODO: Tune

        self.lastTime = rospy.get_time()
        self.altitude_controller = PID(Kp=2.5, Kd=7.5, maxOut=self.request_velocity)
        self.x_controller = PID(Kp=2.5, Kd=7.5, maxOut=self.request_velocity)
        self.y_controller = PID(Kp=2.5, Kd=7.5, maxOut=self.request_velocity)

        self.line_controller_x = PID(Kp=1.5, Kd=5.0, maxOut=0.25)
        self.line_controller_y = PID(Kp=1.5, Kd=5.0, maxOut=0.25)
        self.line_controller_z = PID(Kp=1.5, Kd=5.0, maxOut=0.25)

    def update_line(self, current_state, request_velocity):
        self.state = np.array([current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z])
        self.x_controller.maxOut = request_velocity
        self.y_controller.maxOut = request_velocity
        self.altitude_controller.maxOut = request_velocity
        #vector_between_A_and_B = self.point_B - self.point_B

        closest_point, distances, actual_distance = self.closest_point_on_vector(self.point_A[0], self.point_A[1], self.point_A[2], self.point_B[0], self.point_B[1], self.point_B[2], self.state[0], self.state[1], self.state[2])
        # Output vector is now just the distance between the current position and the closest position to the vector.
        #output_vector = closest_point - self.state
        #output_vector = distances # Maybe this is correct... I'm not sure
        output_vector = np.array([0, 0, 0])
        # We have to "scale it" 

        '''
        # We have to create some "Target" for the PID controller as the target B and the direction of the unit vector do not match very well.
        # What we are going to do is push the drone towards the target B as much as we can allow it to depending on how close we are to the line.
        # We are going to represent "closeness" as the remainder of 1 minus the vector (not unit vector) from the drone to the line, if it is under 1 at all.
        # We are going to use that remainder as how much we can push the drone towards the goal B. Ideally, when the drone is on the line perfectly, it is flying in a unit direction towards the goal.
        # If it's more than one meter away from the line in any direction, all it tries to do is get back to it, and does not focus on actually getting to the goal B.
        remainder_vector = np.array([0, 0, 0])
        if actual_distance < 1.0:
            remainder = 1.0 - actual_distance
            remainder_vector = self.vector_AB_unit * remainder
            # Now that the output_vector is not a unit vector but a small vector pointing on the line, we add the remainder-contribution.
            output_vector += remainder_vector
            # This is still not necessarily a unit vector, so we make it one.
            output_vector = output_vector / (output_vector[0]**2 + output_vector[1]**2 + output_vector[2]**2)**.5

        else :
            # Normalizing the output vector
            output_vector = output_vector / actual_distance
        '''

        # When the distance is closer than 1 meter to the line, the unit vector towards the line actually "goes through" the line. So what we do is skip the normalization when that is the case, and instead use vector addition
        time = rospy.get_time()
        x_line_factor = self.line_controller_x.update(closest_point[0], self.state[0], time)
        y_line_factor = self.line_controller_x.update(closest_point[1], self.state[1], time)
        z_line_factor = self.line_controller_x.update(closest_point[2], self.state[2], time)

        time = rospy.get_time()
        x_factor = self.x_controller.update(self.target[0], self.state[0], time)
        y_factor = self.y_controller.update(self.target[1], self.state[1], time)
        altitude_factor = self.altitude_controller.update(self.target[2], self.state[2], time)
        
        # Multiplying each controller factor on the vector component it corresponds to...
        output_vector[0] = x_factor + x_line_factor
        output_vector[1] = y_factor + y_line_factor
        output_vector[2] = altitude_factor + z_line_factor

        # normalize or not?
        #output_vector = output_vector / (output_vector[0]**2 + output_vector[1]**2 + output_vector[2]**2)**.5

        
        output = Twist()
        output.linear.x = output_vector[0]
        output.linear.y = output_vector[1]
        output.linear.z = output_vector[2]

        output.angular.z = 0

        return output
    
    def update_point(self, current_state, request_velocity):
        self.state = np.array([current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z, current_state.pose.orientation.z])
        self.x_controller.maxOut = request_velocity
        self.y_controller.maxOut = request_velocity
        self.altitude_controller.maxOut = request_velocity

        # Drawing a vector from our current position to a target position
        output_vector = np.empty(3)
        output_vector[0] = self.target[0] - self.state[0]
        output_vector[1] = self.target[1] - self.state[1]
        output_vector[2] = self.target[2] - self.state[2]

        # Normalizing vector
        output_vector = output_vector / (output_vector[0]**2 + output_vector[1]**2 + output_vector[2]**2)**.5

        time = rospy.get_time()
        
        x_factor = self.x_controller.update(self.target[0], self.state[0], time)
        y_factor = self.y_controller.update(self.target[1], self.state[1], time)
        altitude_factor = self.altitude_controller.update(self.target[2], self.state[2], time)
        
        # Multiplying each controller factor on the vector component it corresponds to...
        output_vector[0] *= abs(x_factor)
        output_vector[1] *= abs(y_factor)
        output_vector[2] *= abs(altitude_factor)

        #string = "x: " + str(output_vector[0]) + " y: " + str(output_vector[1]) + " z: " + str(output_vector[2])
        #rospy.loginfo_throttle(0.5, string)

        # Creating the output Twist message
        output = Twist()
        output.linear.x = output_vector[0]
        output.linear.y = output_vector[1]
        output.linear.z = output_vector[2]

        # Testing just raw PID instead
        #output.linear.x = x_factor
        #output.linear.y = y_factor
        #output.linear.z = altitude_factor

        # TODO: Yaw controller

        # The current_state.orientation is a Pose message, and therefore actually a quaternion, which means...
        # ... that we have to translate the quaternion into euler form so that we can compare the two correctly for the PID controller :) 
        current_quat = Quaternion( current_state.pose.orientation.w, current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z)
        current_euler = current_quat.to_euler()

        # Since the PID controller calculate the error itself and I dont wanna custimize it, I ghetto fix and create a fake error myself with the rules that apply for the yaw
        error = (current_euler[2] + math.pi) -  self.target[3]
        #error = (error + 180) % 360 - 180
        if error > math.pi:
            error -= math.pi*2
        elif error < -math.pi:
            error += math.pi*2

    
        yaw_input = self.Yaw.update(0, error, time)

        #rospy.loginfo(self.target[3])
        #rospy.loginfo(current_euler[2])

        output.angular.z = yaw_input

        return output


    def set_target(self, target):
        self.target = np.array([target.position.x, target.position.y, target.position.z, target.orientation.z])

    def set_line(self, A, B):
        self.point_A = np.array([A.x, A.y, A.z])
        self.point_B = np.array([B.x, B.y, B.z])
        self.vector_AB = self.point_B - self.point_A
        self.vector_AB_unit = self.vector_AB / (self.vector_AB[0]**2 + self.vector_AB[1]**2 + self.vector_AB[2]**2)**.5
        self.target = np.array([B.x, B.y, B.z])

    def reached(self, radius):
        if ((self.target[2] - radius) < self.state[2] < (self.target[2] + radius) 
            and (self.target[1] - radius) < self.state[1] < (self.target[1] + radius) 
            and (self.target[0] - radius) < self.state[0] < (self.target[0] + radius)):
            return True
        else:
            return False
        # TODO: yaw reached
        #and (self.target[3] - 0.05) < self.state[3] < (self.target[3] + 0.05)
        

    def closest_point_on_vector(self, x1, y1, z1, x2, y2, z2, x3, y3, z3): # x3,y3,z3 is the point
        px = x2-x1
        py = y2-y1
        pz = z2-z1

        norm = px*px + py*py + pz*pz

        u =  ((x3 - x1) * px + (y3 - y1) * py + (z3 - z1) * pz) / float(norm)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * px
        y = y1 + u * py
        z = z1 + u * pz

        dx = x - x3
        dy = y - y3
        dz = z - z3

        dist = (dx*dx + dy*dy + dz*dz)**.5

        return np.array([x, y, z]), np.array([dx, dy, dz]), dist


'''
class VelocityController:
    target = PoseStamped()
    output = Twist()

    def __init__(self):
        self.X = PID(maxOut=2)
        self.Y = PID(maxOut=2)
        self.Z = PID(Kp = 1.5, Kd=2.5, maxOut=2)
        self.Yaw = PID(maxOut=0.5)

        self.lastTime = rospy.get_time()
        self.target = None

    def set_target(self, target):
        self.target = target

    def update(self, state):
        if (self.target is None):
            rospy.logwarn("Target position for velocity controller is none.")
            return None
        # simplify variables a bit
        time = state.header.stamp.to_sec()
        position = state.pose.position
        orientation = state.pose.orientation
        # create output structure
        output = Twist()

        # output velocities
        linear = Vector3()
        angular = Vector3()
        # Control in X vel
        linear.x = self.X.update(self.target.position.x, position.x, time)
        # Control in Y vel
        linear.y = self.Y.update(self.target.position.y, position.y, time)
        # Control in Z vel
        linear.z = self.Z.update(self.target.position.z, position.z, time)
        # Control yaw (no x, no y, just yaw)
        angular.z = self.Yaw.update(self.target.orientation.z, orientation.z, time)

        # TODO
        output.linear = linear
        output.angular = angular



        return output
'''