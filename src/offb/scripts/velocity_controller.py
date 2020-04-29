# Thanks to darknight-007 and his docking repos on GitHub

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Twist, TwistStamped
import math
import numpy as np
from std_msgs.msg import Header
from PID import PID



class VelocityGuidance:
    output = Twist()

    def __init__(self, request_velocity = 1.0):
        self.point_A = np.array([])
        self.point_B = np.array([])

        self.request_velocity = request_velocity


        self.state = np.array([])

        self.target = np.array([])

        self.Yaw = PID(Kp=1.75, Kd=6, maxOut=0.5)


        self.lastTime = rospy.get_time()
        self.altitude_controller = PID(Kp=2.5, Kd=7.5, maxOut=self.request_velocity)

    def update_line(self, current_state):
        self.state = np.array([current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z])
        
        #vector_between_A_and_B = self.point_B - self.point_B

        closest_point, distance = self.closest_point_on_vector(self.point_A[0], self.point_A[1], self.point_A[2], self.point_B[0], self.point_B[1], self.point_B[2], self.state[0], self.state[1], self.state[2])
        # Output vector is now just the distance between the current position and the closest position to the vector.
        output_vector = closest_point - self.state
        # We have to "scale it" 

        # Pushing it towards the goal, point_B
        output_vector += (self.target - self.state)

        # Normalizing the output vector
        output_vector = output_vector / distance

        # Multiplying it with the requested speed
        output_vector *= self.request_velocity
        
        output = Twist()
        output.linear.x = output_vector[0]
        output.linear.y = output_vector[1]
        output.linear.z = output_vector[2]

        output.angular.z = 0

        return output
    
    def update_point(self, current_state):
        self.state = np.array([current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z])

        # Drawing a vector from our current position to a target position
        output_vector = self.target - self.state

        # Normalizing vector
        output_vector = output_vector / (output_vector[0]**2 + output_vector[1]**2 + output_vector[2]**2)**.5

        time = rospy.get_time()
        # Let's fake the error and make it scalar of the distance between here and there
        
        altitude_factor = self.altitude_controller.update(self.target[2], self.state[2], time) #TODO: Make all the controllers individual.
        # Scaling it down to the current request speed
        output_vector *= altitude_factor

        #rospy.loginfo(str(output_vector[2]))

        output = Twist()
        output.linear.x = 0#output_vector[0]
        output.linear.y = 0#output_vector[1]
        output.linear.z = output_vector[2]

        output.angular.z = 0

        return output


    def set_target(self, target):
        self.target = np.array([target.position.x, target.position.y, target.position.z])

    def set_line(self, A, B):
        self.point_A = np.array([A.x, A.y, A.z])
        self.point_B = np.array([B.x, B.y, B.z])


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

        # Note: If the actual distance does not matter,
        # if you only want to compare what this function
        # returns to other results of this function, you
        # can just return the squared distance instead
        # (i.e. remove the sqrt) to gain a little performance

        dist = (dx*dx + dy*dy + dz*dz)**.5

        return Point(x, y, z), dist



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
