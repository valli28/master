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



        self.offset = 8.0 # m

        
    def draw_mission_planes(self, list_of_planes, axes):
        
        for i in range(len(list_of_planes)):
            # Fetch the planes of the building and offset them with the normal of that plane
            newX = list_of_planes[i].X - list_of_planes[i].normal[0] * self.offset
            newY = list_of_planes[i].Y - list_of_planes[i].normal[1] * self.offset
            # Plot the new offset plane
            axes.plot_surface(newX, newY, list_of_planes[0].Z, alpha=0.9, color = 'b')

    def generate_mission(self):
        print("generate the mission here")

        

