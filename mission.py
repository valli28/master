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
        self.mat_planes = []
        self.colortuple = ('r', 'b')
        self.tiles_affected_by_reflection = [] 
        self.affected_tiles_angles = []

        self.offset = 8.0 # m

    def check_for_reflection(self, sun, axes):
        # sun.roa is a n x 6 shape array where n is the number of vectors in the program
        intersection_point = [0, 0 , 0]
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

                if d > 0: # If the d is zero, that means that the vector isn't drawn, or if the vector and the plane are parallel/contained in the plane
                    # The point of intersection can now be calculated
                    intersection_point = l0 + l * d
                
                
                # Now to see if the intersection point is contained within our plane-segment
                lenX, lenY, lenZ = len(self.mission_planes[j].X) - 1 , len(self.mission_planes[j].Y) - 1 , len(self.mission_planes[j].Z) - 1 
                if np.count_nonzero(intersection_point) != 0 and d != 0:
                    if (self.mission_planes[j].Z[0][0] < intersection_point[2] < self.mission_planes[j].Z[lenZ][0] or self.mission_planes[j].Z[0][0] > intersection_point[2] > self.mission_planes[j].Z[lenZ][0]) and \
                       (self.mission_planes[j].X[0][0] < intersection_point[0] < self.mission_planes[j].X[0][lenX] or self.mission_planes[j].X[0][0] > intersection_point[0] > self.mission_planes[j].X[0][lenX]) and \
                       (self.mission_planes[j].Y[0][0] < intersection_point[1] < self.mission_planes[j].Y[0][lenY] or self.mission_planes[j].Y[0][0] > intersection_point[1] > self.mission_planes[j].Y[0][lenY]):

                        x_index = np.argmin(self.mission_planes[j].X[0] < intersection_point[0]) - 1 # This returns the index of the first tile where a ray is reflected upon
                        #y_index = np.argmax(self.mission_planes[j].Y[0] < intersection_point[1]) # I only have to find either the x or y index, since they describe the same thing in that dimension # TODO: maybe not for roofs or other kinds of planes?
                        z_index = np.argmin(np.transpose(self.mission_planes[j].Z)[0] < intersection_point[2]) - 1 # Since the z-plane is transposed compared to x and y planes, I have to index it in a different way.
                        
                        # These indices are used to overwrite the color of the new face-color-map for the plane in question.
                        self.tiles_affected_by_reflection[j][z_index][x_index] = 'r'


                        # We store the first angle of reflection
                        if self.affected_tiles_angles[j][0] == [0, 0, 0]: # This condition is no-go
                            self.affected_tiles_angles[j][0] = [l[0], l[1], l[2]]
                        # but we continue to overwrite the last angle until this program no longer runs, and the path-generation takes over.
                        self.affected_tiles_angles[j][1] = [l[0], l[1], l[2]] # This becomes zero becuase.... the l is zero sometimes when it's off
                        #print("plane " + str(j) +" has " + str(self.affected_tiles_angles[j]))

                        # First we remove the plane we are working on and then create a new one from the mission-planes with the new colors.
                        self.mat_planes.pop(j).remove()
                        self.mat_planes.insert(j, axes.plot_surface(self.mission_planes[j].X, self.mission_planes[j].Y, self.mission_planes[j].Z, alpha=0.8, facecolors = self.tiles_affected_by_reflection[j], linewidth = 0))


    def draw_mission_planes(self, list_of_planes, axes):
        
        self.mission_planes = list_of_planes.copy()

        for i in range(len(list_of_planes)):          
            # Fetch the planes of the building and offset them with the normal of that plane
            newX = list_of_planes[i].X - list_of_planes[i].normal[0] * self.offset
            newY = list_of_planes[i].Y - list_of_planes[i].normal[1] * self.offset

            self.mission_planes[i].X = newX
            self.mission_planes[i].Y = newY

            # Make a checker pattern plot to see how you make a facecolor-map

            colors = np.empty(list_of_planes[i].X.shape, dtype=str)
            for y in range(len(list_of_planes[i].y)):
                for x in range(len(list_of_planes[i].x)):
                    colors[x, y] = self.colortuple[1]
            #print(colors)
            face_color_array = colors

            # Plot the new offset plane
            self.mat_planes.append(axes.plot_surface(newX, newY, list_of_planes[0].Z, alpha=0.8, facecolors = face_color_array, linewidth = 0))
        
        self.tiles_affected_by_reflection =  [None] * len(self.mission_planes)
        #print(self.tiles_affected_by_reflection)
        self.affected_tiles_angles = [None] * len(self.mission_planes)

        for i in range(len(self.tiles_affected_by_reflection)):
            # Initialise the tiles affected as a list of planes.
            self.tiles_affected_by_reflection[i] = np.array(self.mission_planes[0].X, dtype=str)
            self.tiles_affected_by_reflection[i][:] = self.colortuple[1]

            # Initialize the affected tiles-angles (for each plane we have the first and last angle at which they are affected.) (might not be scalable in terms of the amount of windows... if they are angled differently.)
            self.affected_tiles_angles[i] = [[0, 0, 0], [0, 0, 0]]



    def generate_mission(self, axes):

        # TODO: calculate the first and the last angle at which the tiles were affected. 
        # Let's say that the mission takes 30 minutes. Thus the sun moves a bit, and the angle at which the sun's reflection affects the tiles varies throughout the flight.
        # These angles are necessary to determine whether the mission-tiles in question are affected with the camera (fov, gsd, etc.) in question. 

        # Appearently, no-one has implemented the brushfire/wavefront algirithm that I want and uploaded it on github or anywhere, so I'll have to do it myself. 
        # This algorithm "ignites" the obstacles (the border of the mission-plane and lines that I insert to get the path that I want)
        # For the purpose of development, I'll make a new figure in matplotlib, which is a 2D grid image of the mission-plane

        copy_tiles = self.tiles_affected_by_reflection.copy()

        for i in range(len(copy_tiles)):
            # The copy-tiles is a list of n-planes with strings, representing colors. b for blue, r for red etc.
            # First, we insert convert that char-plane into a integer-plane, where b is 0 and r is something high that we then use to count down from.

            integer_tile_map = np.zeros_like(copy_tiles[i], dtype=int)
            integer_tile_map = (copy_tiles[i] == 'r') * 1
            # The integer_tile_map is now a single plane in this loop that is 1 for red and 0 for b.

            # make a border in the integer_tile_map of 1's
            # rows:
            integer_tile_map[0:1] = 1
            integer_tile_map[len(integer_tile_map) - 1:len(integer_tile_map)] = 1
            # columns:
            integer_tile_map[:, 0] = 1
            integer_tile_map[:, len(integer_tile_map) - 1] = 1

            # We now have to insert lines into the plane that shall help form the correct paths. 
            # The distance and lengths of these lines have to be calculated from parameters such as fov, gsd, etc.
            


            print(integer_tile_map)

            


            

        

