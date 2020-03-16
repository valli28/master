import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

class Mission():
    def __init__(self, l_overlap, h_overlap):
        # Initialize parameters from constructor (related to the mission)
        self.l_overlap = l_overlap # in percentages
        self.h_overlap = h_overlap # in percentages


        # Initialize constant parameters such as camera, drone stuff etc.
        self.fov = 75 # degrees
        self.sensor_size = np.array([21, 16]) # mm

        
    def draw_mission_planes(self, list_of_planes):
        print("matplotlib stuff here")


    def generate_mission(self):
        print("generate the mission here")

        

