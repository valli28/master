from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt



class Mission():
    time_of_day = 9.5
    planned_flight_duration = 0.5

    def __init__(self):
        print("mission created")

class Drone():
    m = 3.0

    # size, battery life etc.

class Camera():
    fov = 80 #deg
    sensor_size = 25 #mm
    # All the different parameters like fov, resolution, sensor size etc.


class Surface():
    # Material = emissivity etc. 
    # coordinates, windows?
    # Can also be a roof

    def __init__(self, points):
        self.points = points

class Structure():
    
    def __init__(self, surfaces):
        self.surfaces = surfaces
    # Contains more walls

    # Function in here that finds points in surfaces that are the same and "joins" them. 

fig = plt.figure()
ax = Axes3D(fig)
x = [0,1,1,0]
y = [0,0,0,0]
z = [0,0,1,1]
wall1 = [list(zip(x,y,z))]


ax.add_collection3d(Poly3DCollection(wall1))
plt.show()