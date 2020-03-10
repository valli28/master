import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import cm
from pylab import meshgrid,linspace,zeros,dot,norm,cross,vstack,array,matrix,sqrt


# Define boundary as 3 points to create a plane. The points create two vectors, p01 and p02 that span the plane. 
class Boundary():
    def __init__(self, x0, y0, z0, x1, y1, z1, x2, y2, z2): # Add more points. This is supposed to be a polygon. 

        ## Geometric parameters #########################################
        # Points
        self.p0 = np.array([x0, y0, z0])
        self.p1 = np.array([x1, y1, z1])
        self.p2 = np.array([x2, y2, z2])

        self.x = np.array([x0, x1, x2])
        self.y = np.array([y0, y1, y2])
        self.z = np.array([z0, z1, z2])

        # Vectors
        self.p01 = self.p1 - self.p0
        self.p02 = self.p2 - self.p0
        self.cross = np.cross(self.p02, self.p01)
        self.normal = self.cross / np.linalg.norm(self.cross)


        # Calculate plane
        #self.X, self.Y, self.Z = self.plane(np.linalg.norm(self.p01), np.linalg.norm(self.p02), 20, 20, self.normal, self.z[2])


        Nx = Ny = Nz = 10
        self.X, self.Y, self.Z = self.plane_matrices(Nx, Ny, Nz)

        '''
        # a plane is a*x+b*y+c*z+d=0
        # [a,b,c] is the normal. Thus, we have to calculate
        # d and we're set
        d = -self.p0.dot(self.normal)

        # xx and yy are boundaries for how far the plane goes in the plot... (I think..)
        # Should probabyl be the length of p01 and p02 vectors, but it doesn't like non-integer values
        self.xx, self.yy = np.meshgrid(range(5), range(5))
        if(self.normal[2] != 0.0):
            self.zz = (-self.normal[0] * self.xx - self.normal[1] * self.yy - d) * (1.0 / self.normal[2])
        else:
            self.zz = self.yy * 0
        '''
 
        ## Material parameters #########################################
        self.reflectiveness = 1.0 # Default reflectiveness is 100% (mirror)


    def plane_matrices(self, Nx, Ny, Nz):
        """ Calculate points of a generic plane 
        Arguments:
        - `Nx` : Number of points, first direction
        - `Ny` : Number of points, second direction
        - `Nz` : Number of points, third direction

        >>> x = np.arange(-5, 5, 0.1)
        >>> y = np.arange(-5, 5, 0.1)
        >>> xx, yy = np.meshgrid(x, y, sparse=True)
        >>> z = np.sin(xx**2 + yy**2) / (xx**2 + yy**2)
        """

        # Based on the points and vectors, we draw planes that make sense. The matrices are not solved based on eachother or some plane formula due to divide-by-zero 

        x = np.linspace(self.p0[0], self.p1[0], Nx)
        y = np.linspace(self.p0[1], self.p1[1], Ny)
        z = np.linspace(self.p0[2], self.p2[2], Nz)


        print("xyz's")
        print(x)
        print(y)
        print(z)
        
        xx, yy = np.meshgrid(x, y)
        zz,  = np.meshgrid(z)

        print("xxyyzz's")
        print(xx)
        print(yy)
        print(zz)
        
    
        return xx, yy, zz


    def plot(self, axes):
        axes.plot_surface(self.X, self.Y, self.Z, rstride=5, cstride=5, alpha=0.8)
        axes.plot(*zip(self.p0, self.p1, self.p2), color='r', linestyle=' ', marker='o')

