from pysolar.solar import *
import datetime
import numpy as np
import utm
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt


#Import custom classes
from sun import Sun
from boundary import Boundary
#s = Sun()
#print("Distance between earth and sun is currently = " + str(s.distance_earth_sun))

coord1 = [10.32522211465866,55.47169437449494]
coord2 = [10.32524982538288,55.47166787993427]
coord3 = [10.32531271941916,55.47168811627513]
coord4 = [10.32528783658924,55.47171798510018]

utm1 = utm.from_latlon(coord1[1], coord1[0])
utm2 = utm.from_latlon(coord2[1], coord2[0])
utm3 = utm.from_latlon(coord3[1], coord3[0])
utm4 = utm.from_latlon(coord4[1], coord4[0])

lat = 55.471689
lon = 10.325267
utm_center = utm.from_latlon(lat, lon)

local1 = [utm_center[0] - utm1[0], utm_center[1] - utm1[1]]
local2 = [utm_center[0] - utm2[0], utm_center[1] - utm2[1]]
local3 = [utm_center[0] - utm3[0], utm_center[1] - utm3[1]]
local4 = [utm_center[0] - utm4[0], utm_center[1] - utm4[1]]

height = 2.0
east_wall = Boundary(local1[0], local1[1], 0.0, local2[0], local2[1], 0.0, local1[0], local1[1], height)
north_wall = Boundary(local2[0], local2[1], 0.0, local3[0], local3[1], 0.0, local2[0], local2[1], height)
west_wall = Boundary(local3[0], local3[1], 0.0, local4[0], local4[1], 0.0, local3[0], local3[1], height)
south_wall = Boundary(local4[0], local4[1], 0.0, local1[0], local1[1], 0.0, local4[0], local4[1], height)



fig = plt.figure(num=1, clear=True)
ax = fig.add_subplot(1, 1, 1, projection='3d')


east_wall.plot(ax)
north_wall.plot(ax)
west_wall.plot(ax)
south_wall.plot(ax)

ax.set_xlabel('X')
ax.set_xlim(-10, 10)
ax.set_ylabel('Y')
ax.set_ylim(-10, 10)
ax.set_zlabel('Z')
ax.set_zlim(-10, 10)

plt.show()
