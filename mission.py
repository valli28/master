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

        self.mission_planes = []

        self.offset = 8.0 # m

    def check_for_reflection(self, sun, axes):
        # sun.roa is a n x 6 shape array where n is the number of vectors in the program

        intersection_point = [0, 0 , 0]
        correct_intersection_points = []
        for j in range(len(self.mission_planes)):
            for i in range(len(sun.roa)):
                #print(self.mission_planes[0].X[0][0])
                p0 = np.array([self.mission_planes[j].X[0][0], self.mission_planes[j].Y[0][0], self.mission_planes[j].Z[0][0]]) # a point in the plane
                l0 = np.array([sun.roa[i][0], sun.roa[i][1], sun.roa[i][2]]) # A point on the line
                l = np.array([sun.roa[i][3], sun.roa[i][4], sun.roa[i][5]]) # A vector pointing in the direction of the line
                n = self.mission_planes[j].normal # The normal of the plane

                if np.count_nonzero(l) == 0: # If the vector is not supposed to be drawn, the roa returns 0, 0, 0 vector which screws up the dot product in the d formula.
                    d = 0
                else:
                    d = np.dot((p0-l0), n) / np.dot(l, n)

                if d != 0: # If the d is zero, that means that the vector isn't drawn, or if the vector and the plane are parallel/contained in the plane
                    # The point of intersection can now be calculated
                    intersection_point = l0 + l * d
                
                
                    
                # Now to see if the intersection point is contained within our plane-segment
                lenX, lenY, lenZ = len(self.mission_planes[j].X) - 1 , len(self.mission_planes[j].Y) - 1 , len(self.mission_planes[j].Z) - 1 
                if np.count_nonzero(intersection_point) != 0:
                    if (self.mission_planes[j].Z[0][0] < intersection_point[2] < self.mission_planes[j].Z[lenZ][0] or self.mission_planes[j].Z[0][0] > intersection_point[2] > self.mission_planes[j].Z[lenZ][0]) and \
                       (self.mission_planes[j].X[0][0] < intersection_point[0] < self.mission_planes[j].X[0][lenX] or self.mission_planes[j].X[0][0] > intersection_point[0] > self.mission_planes[j].X[0][lenX]) and \
                       (self.mission_planes[j].Y[0][0] < intersection_point[1] < self.mission_planes[j].Y[0][lenY] or self.mission_planes[j].Y[0][0] > intersection_point[1] > self.mission_planes[j].Y[0][lenY]):
                        #axes.scatter3D(intersection_point[0], intersection_point[1], intersection_point[2])
                        correct_intersection_points.append(intersection_point)
        
            # We now use the correct intersection points to make a new color-map.
            colortuple = ('y', 'b')
            colors = np.empty(list_of_planes[i].X.shape, dtype=str) 
            for y in range(len(list_of_planes[i].y)): # y and x values are outdated for these planes, but since we are only using them to iterate, it's okay.
                for x in range(len(list_of_planes[i].x)):
                    # 300 IQ one-liner to color the faces of affected tiles.
                    # if the point is inside any square, color that square.
                    # Just a print so that the compiler doesn't cry
                    print("heeh") 

            # We have to delete the old plane somehow if I don't want to melt my PC...
            axes.plot_surface(newX, newY, list_of_planes[0].Z, alpha=0.9, facecolors = face_color_array, linewidth = 0)

        
        #print("Next")
                        




        
    def draw_mission_planes(self, list_of_planes, axes):
        
        self.mission_planes = list_of_planes.copy()

        for i in range(len(list_of_planes)):          
            # Fetch the planes of the building and offset them with the normal of that plane
            newX = list_of_planes[i].X - list_of_planes[i].normal[0] * self.offset
            newY = list_of_planes[i].Y - list_of_planes[i].normal[1] * self.offset

            self.mission_planes[i].X = newX
            self.mission_planes[i].Y = newY

            # Make a checker pattern plot to see how you make a facecolor-map
            colortuple = ('y', 'b')
            colors = np.empty(list_of_planes[i].X.shape, dtype=str)
            for y in range(len(list_of_planes[i].y)):
                for x in range(len(list_of_planes[i].x)):
                    colors[x, y] = colortuple[(x + y) % len(colortuple)]
            face_color_array = colors

            # Plot the new offset plane
            axes.plot_surface(newX, newY, list_of_planes[0].Z, alpha=0.9, facecolors = face_color_array, linewidth = 0)

    def generate_mission(self):
        print("generate the mission here")

        

