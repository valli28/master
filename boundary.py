import numpy as np

# Define boundary as 3 points to create a plane. The points create two vectors, p01 and p02 that span the plane. 
class Boundary():
    def __init__(self, x0, y0, z0, x1, y1, z1, x2, y2, z2): # Add more points. This is supposed to be a polygon. 
        # Points
        self.p0 = np.array([x0, y0, z0])
        self.p1 = np.array([x1, y1, z1])
        self.p2 = np.array([x2, y2, z2])

        self.x = np.array([x0, x1, x2])
        self.y = np.array([y0, y1, y2])
        self.z = np.array([z0, z1, z2])

        # Vectors
        p01 = self.p1 - self.p0
        p02 = self.p2 - self.p0

        self.reflectiveness = 1.0 # Default reflectiveness is 100% (mirror)


    def plot(self, axes):
        axes.plot_trisurf(self.x, self.y, self.z, linewidth=0, antialiased=False)

