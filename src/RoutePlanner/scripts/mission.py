import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import math
from skimage.morphology import skeletonize, thin

class Mission():
    def __init__(self, l_overlap, h_overlap):
        self.mission_planes = []
        self.mat_planes = []
        self.colortuple = ('r', 'b')
        self.tiles_affected_by_reflection = [] 
        self.affected_tiles_angles = []

        
        # Initialize parameters from constructor (related to the mission)
        self.l_overlap = l_overlap / 100 # in percentages
        self.h_overlap = h_overlap / 100 # in percentages


        # Ideally, the "offset" from the building is enough to accomodate for multipathing and other bad influences from being close to buildings (magnetometer etc.)
        # If the purpose of the flight is to find cracks in the material, you would have to find a camera with a higher resolution or maybe a narrower FOV.
        # Let's try to see what happens if we want to say "I want to see "sub-brick details" like differences in betwee the bricks (mortar)" 
        # After a very quick survey, the distance between the bricks in my home is anything between 1cm and 3cm. A Ground Sampling Distance of >1cm is ideal.
        # Let's see if that is possible
        self.GSD = 1.0 #cm per pixel. Less is better (higher ground-resolution)
        self.GSD *= 0.5 # Due to Nyquist's sampling theorem i guess..

        '''
        ################################################################################
        # For the FLIR Duo Pro R (not the camera I used. It's discontinued and I couldn't find nor calculate the focal length anywhere...)
        # The FLIR Duo Pro R camera can be purchased in many different variations, both in terms of thermal resolution, FOV and refresh-rate.
        #Let's take the best resolution, no matter the refresh-rate and middle-range FOV.
        # Initialize constant parameters such as camera, drone stuff etc.
        sensor_resolution = np.array([640, 512])# pixels

        self.aov = np.array([25.0, 20.0])  # degrees so they say that the AOV is 32 degrees
        #diag_fov = math.hypot(lens[0], lens[1]) #calculating the hypotenuse

        pixel_pitch = 17.0 / 1000 # pixel pitch (distance between one pixel core to the next) is 17 micrometers
        sensor_size = sensor_resolution * pixel_pitch

        #sensor_size_diagonal = math.hypot(sensor_size[0], sensor_size[1])
        f = 25 # focal length in mm.
        ################################################################################
        '''

        ################################################################################
        # For the CGO3 camera
        # Initialize constant parameters such as camera, drone stuff etc.
        self.sensor_resolution = np.array([1920, 1080])# pixels

        self.aov = np.array([114.592, 114.592*(self.sensor_resolution[1]/self.sensor_resolution[0])])  # deg
        #diag_fov = math.hypot(lens[0], lens[1]) #calculating the hypotenuse

        sensor_size = np.array([25.4, 25.4])

        #sensor_size_diagonal = math.hypot(sensor_size[0], sensor_size[1])
        self.f = 14 # focal length in mm.
        ################################################################################

        ################################################################################
        # For the FLIR Duo Pro R (not the camera I used. It's discontinued and I couldn't find nor calculate the focal length anywhere...)
        # The FLIR Duo Pro R camera can be purchased in many different variations, both in terms of thermal resolution, FOV and refresh-rate.
        #Let's take the best resolution, no matter the refresh-rate and middle-range FOV.
        # Initialize constant parameters such as camera, drone stuff etc.
        thermal_sensor_resolution = np.array([640, 512])# pixels

        self.thermal_aov = np.array([25.0, 20.0])  # degrees so they say that the AOV is 32 degrees
        #diag_fov = math.hypot(lens[0], lens[1]) #calculating the hypotenuse

        pixel_pitch = 17.0 / 1000 # pixel pitch (distance between one pixel core to the next) is 17 micrometers
        thermal_sensor_size = thermal_sensor_resolution * pixel_pitch

        #sensor_size_diagonal = math.hypot(sensor_size[0], sensor_size[1])
        thermal_f = 25 # focal length in mm.
        ################################################################################
    
        self.h = np.round(min((thermal_f * thermal_sensor_resolution * self.GSD) * 1.0 / thermal_sensor_size) / 100.0, 2) # Divide by 100 to get meters from cm(GSD).
        print("With a minimum GSD of " + str(self.GSD) + "cm the UAV can fly at a maximum distance of " + str(self.h) + "m from the walls")
        self.thermal_fov = 2.0 * self.h * (np.tan(np.radians(self.thermal_aov) / 2)) # This is the "field" that the camera can see. In the object-plane, how much does the x and y axes see. 2 (TAN (ANGLE OF VIEW/2) X DISTANCE TO SUBJECT) 
        print("The thermal FOV of the thermal camera is" + str(self.thermal_fov))
        self.camera_pitch = self.thermal_aov[1] * 0.6  # in radians

        #self.altitude = (math.sin(self.camera_pitch) * self.h) / math.sin(90.0 - self.camera_pitch) # 
        #print("... and it will by flying at a minimum altitude of " + str(self.altitude) + "m")

    def check_for_reflection(self, sun, axes):
        # sun.roa is a n x 6 shape array where n is the number of vectors in the program
        intersection_point = [0, 0, 0]
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
                lenX = len(self.mission_planes[j].X[0]) - 1 
                lenY = len(self.mission_planes[j].Y[0]) - 1
                lenZ = len(self.mission_planes[j].Z) - 1 
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
            newX = list_of_planes[i].X - list_of_planes[i].normal[0] * self.h
            newY = list_of_planes[i].Y - list_of_planes[i].normal[1] * self.h

            self.mission_planes[i].X = newX
            self.mission_planes[i].Y = newY

            # Find how much the mission planes should be lifted
            vertical_offset = self.h * math.sin(math.radians(self.aov[1]*0.5)) / math.sin(math.radians(90 - self.aov[1]*0.5))
            self.mission_planes[i].Z = list_of_planes[i].Z + vertical_offset

            # Make a checker pattern plot to see how you make a facecolor-map
            colors = np.empty(list_of_planes[i].X.shape, dtype=str)
            for z in range(len(list_of_planes[i].z)):
                for x in range(len(list_of_planes[i].x)):
                    colors[z, x] = self.colortuple[1]
            #print(colors)
            face_color_array = colors

            # Plot the new offset plane
            self.mat_planes.append(axes.plot_surface(newX, newY, self.mission_planes[i].Z, alpha=0.8, facecolors = face_color_array, linewidth = 0))
        
        self.tiles_affected_by_reflection =  [None] * len(self.mission_planes)
        #print(self.tiles_affected_by_reflection)
        self.affected_tiles_angles = [None] * len(self.mission_planes)

        for i in range(len(self.tiles_affected_by_reflection)):
            # Initialise the tiles affected as a list of planes.
            self.tiles_affected_by_reflection[i] = np.array(self.mission_planes[i].X, dtype=str)
            self.tiles_affected_by_reflection[i][:] = self.colortuple[1]

            # Initialize the affected tiles-angles (for each plane we have the first and last angle at which they are affected.) (might not be scalable in terms of the amount of windows... if they are angled differently.)
            self.affected_tiles_angles[i] = [[0, 0, 0], [0, 0, 0]]
            # TODO: Use these angles to make sure that the AOV of the camera is taken into consideration. If rays are being reflected towards the camera froma source that is out of frame, then it doesn't matter.

    def draw_mission_snake(self, tile_map):
        #invert the tile-map 
        tile_map = 1 - np.flip(tile_map, 0)
        fig, axes = plt.subplots(figsize=(8, 4))

        axes.imshow(tile_map, cmap=plt.cm.gray)
        axes.set_title('Facade route')
        plt.xlabel('Horizontal [m/2]')
        plt.ylabel('Vertical [m/2]')
        plt.show()

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
            #integer_tile_map[0:1] = 1
            #integer_tile_map[len(integer_tile_map) - 1:len(integer_tile_map)] = 1
            # columns:
            #integer_tile_map[:, 0] = 1
            #integer_tile_map[:, len(integer_tile_map[1]) - 1] = 1

            # We now have to insert lines into the plane that shall help form the correct paths. 
            # The distance and lengths of these lines have to be calculated from parameters such as fov, gsd, etc.

            # Depending on the shape of the building facade, we either do a Z-type or N-type pattern (side to side or up and down.)
            z_or_n = []
            if len(integer_tile_map[:, 0]) >= len(integer_tile_map[0:]): # if the row-dimension is bigger
                z_or_n = 'z'
            else:
                z_or_n = 'n'

            z_or_n = 'n'
            # Depending on the FOV we should take out any false-positive tiles on the mission-plane i.e. tiles that might be affected by sunlight, but not in the current frame.
            # If the window that affects the tiles in the mission-plane isn't in-frame, it should be disregarded.
            if z_or_n == 'n':
                period =  abs(int(self.thermal_fov[0]*(1.0-self.h_overlap) * 1.0 / 0.1)) + 1
                if period % 2 == 1: #  This has to be an even number
                    period -= 1
                # TODO:  the subtraction term in the amplitude should be divided by 0.1 (the amount of meters in each tile). With the size of this building though, we don't have to do any zigzag...
                amplitude = int(len(integer_tile_map[0:]) - (self.l_overlap*self.thermal_fov[1])) + 1 # We round up by casting to integer and +1

                if amplitude % 2 == 1: # This also has to be an even number
                    amplitude -= 1

                flip = 0
                dif = len(integer_tile_map) - amplitude

                # Vertical lines
                integer_tile_map[dif:amplitude, +int(0.5*period)::period] = 1

                # Horizontal lines
                for j in range(0, len(integer_tile_map[0])-1):   
                    if not flip%2: 
                        integer_tile_map[amplitude, period*j+int(0.5*period):period*j + period+1+int(0.5*period)] = 1
                    else:
                        integer_tile_map[dif, period*j+int(0.5*period):period*j + period+int(0.5*period)] = 1
                    flip += 1

                
                self.draw_mission_snake(integer_tile_map)
                #color_map = np.where(integer_tile_map==1, 'r', 'b')
                #axes.plot_surface(self.mission_planes[i].X, self.mission_planes[i].Y, self.mission_planes[i].Z, alpha=0.8, facecolors = color_map, linewidth = 0)
            #print(integer_tile_map)

            


            

        

