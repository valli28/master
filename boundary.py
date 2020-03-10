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
        p01 = self.p1 - self.p0
        p02 = self.p2 - self.p0
        self.cross = np.cross(p02, p01)
        self.normal = self.cross / np.linalg.norm(self.cross)

        distance = z2

        # Calculate plane
        self.X, self.Y, self.Z = self.plane(np.linalg.norm(p01), np.linalg.norm(p02), 20, 20, self.normal, distance)

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


    def rotmatrix(self, axis, costheta):
        """ Calculate rotation matrix

        Arguments:
        - `axis`     : Rotation axis
        - `costheta` : Rotation angle
        """
        x,y,z = axis
        c = costheta 
        s = sqrt(1-c*c)
        C = 1-c

        R = matrix([[ x*x*C+c,    x*y*C-z*s,  x*z*C+y*s ],
                    [ y*x*C+z*s,  y*y*C+c,    y*z*C-x*s ],
                    [ z*x*C-y*s,  z*y*C+x*s,  z*z*C+c]])

        print(R)
        angle = np.trace(R)

        return R

    def plane(self, Lx, Ly, Nx, Ny, n, d):
        """ Calculate points of a generic plane 

        Arguments:
        - `Lx` : Plane Length first direction
        - `Ly` : Plane Length second direction
        - `Nx` : Number of points, first direction
        - `Ny` : Number of points, second direction
        - `n`  : Plane orientation, normal vector
        - `d`  : distance from the origin
        """

        x = linspace(-Lx/2,Lx/2,Nx)
        y = linspace(-Ly/2,Ly/2,Ny)
        # Create the mesh grid, of a XY plane sitting on the orgin
        X,Y = meshgrid(x,y)
        Z = zeros([Nx,Ny])
        #n0 = array([0,0,1]) # This has to be something else. Maybe point in the same direction as the normal vector? This is stupid...
        #n0 = n / np.linalg.norm(n)
        #print(n0)
        #n0 = self.p0 - self.p2
        
        # Rotate plane to the given normal vector
        costheta = 0
        axis     = self.p1 - self.p0 #array([1, 0, 0]) #self.normal
        rotMatrix = self.rotmatrix(axis,costheta)
        #print(rotMatrix)
        XYZ = vstack([X.flatten(),Y.flatten(),Z.flatten()])
        X,Y,Z = array(rotMatrix*XYZ).reshape(3,Nx,Ny)

        
        dVec = (n/norm(n))*d # Distance vector from origin parallel to normal-vector of plane
        X,Y,Z = X+dVec[0],Y+dVec[1],Z+dVec[2]
        return X,Y,Z


    def plot(self, axes):
        axes.plot_surface(self.X, self.Y, self.Z, rstride=5, cstride=5, alpha=0.8)
        axes.plot(*zip(self.p0, self.p1, self.p2), color='r', linestyle=' ', marker='o')

