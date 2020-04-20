#!/usr/bin/env python3
from pysolar.solar import *
import datetime
import numpy as np
import utm
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import cm


#Import custom classes
from sun import Sun
from boundary import Boundary
from mission import Mission



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

s = Sun(lat, lon)
m = Mission(0.8, 0.8)

local1 = [utm_center[0] - utm1[0], utm_center[1] - utm1[1]]
local2 = [utm_center[0] - utm2[0], utm_center[1] - utm2[1]]
local3 = [utm_center[0] - utm3[0], utm_center[1] - utm3[1]]
local4 = [utm_center[0] - utm4[0], utm_center[1] - utm4[1]]

height = 2.0
east_wall = Boundary(local1[0], local1[1], 0.0, local2[0], local2[1], 0.0, local1[0], local1[1], height, "wall", s.get_lightsource())
south_window = Boundary(0.3857, -2.1862, 0.5, 1.3943, -1.532, 0.8, 0.3857, -2.1862, 0.5+0.6, "window", s.get_lightsource())
north_wall = Boundary(local2[0], local2[1], 0.0, local3[0], local3[1], 0.0, local2[0], local2[1], height, "wall", s.get_lightsource())
fake_east_window = Boundary(2.292, 0.424, 0.8, 1.699, 1.342, 0.8, 2.292, 0.424, 0.8+0.5, "window", s.get_lightsource())
west_wall = Boundary(local3[0], local3[1], 0.0, local4[0], local4[1], 0.0, local3[0], local3[1], height, "wall", s.get_lightsource())
fake_west_window = Boundary(-2.4586, -1.0349, 0.8, -1.9872, -2.0232, 0.8, -2.4586, -1.0349, 0.8+0.5, "window", s.get_lightsource)
south_wall = Boundary(local4[0], local4[1], 0.0, local1[0], local1[1], 0.0, local4[0], local4[1], height, "wall", s.get_lightsource())

#ground_plane = Boundary(-5, -5, 0, 5, -5, 0, -5, 5, 0, "grass", s.get_lightsource())

list_of_reflective_boundaries = [fake_east_window, south_window, fake_west_window]
list_of_walls = [east_wall, north_wall, west_wall, south_wall]


fig = plt.figure(num=1, clear=True)
ax = fig.add_subplot(1, 1, 1, projection='3d')


east_wall.plot(ax)
south_window.plot(ax)
north_wall.plot(ax)
fake_east_window.plot(ax)
west_wall.plot(ax)
fake_west_window.plot(ax)
south_wall.plot(ax)

#ground_plane.plot(ax)

s.cast_on(list_of_reflective_boundaries, ax)
m.draw_mission_planes(list_of_walls, ax)


ax.set_xlabel('X: East')
ax.set_xlim(-10, 10)
ax.set_ylabel('Y: North')
ax.set_ylim(-10, 10)
ax.set_zlabel('Z: Up')
ax.set_zlim(-10, 10)


axnext = plt.axes([0.85, 0.25, 0.10, 0.075])
bnext = Button(axnext, 'Path')


slider_box = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
time_slider = Slider(slider_box, 'UNIX Time', valmin=s.date.timestamp() - 60.0*60.0*12.0, valmax=s.date.timestamp() + 12.0*60.0*60.0, valinit=s.date.timestamp())
def update(val):
    s.date = datetime.datetime.fromtimestamp(time_slider.val, tz=pytz.timezone("Europe/Copenhagen"))
    s.cast_on(list_of_reflective_boundaries, ax)
    fig.canvas.draw_idle()
    m.check_for_reflection(s, ax)


def button_callback(event):
    m.generate_mission(ax)


time_slider.on_changed(update)

bnext.on_clicked(button_callback)

plt.show()


'''
consider doing the following, converting it to a rosnode instead of just a package with scripts


if __name__ == '__main__':
    try:
        oracle = OracleClass(mission_alt=15.0)
        oracle.run()
    except rospy.ROSInterruptException:
        exit(0)
    except KeyboardInterrupt:
        exit(0)


'''

