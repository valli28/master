import numpy as np
from pysolar.solar import *
import datetime
import matplotlib.colors as mcolors
import math

# Instead of calling this class "Sun" it could just be "light-source"
# Since the sun is so far away from earth, it's basicly a laser. A common thing to do in games is just use it as a directional light
# It's actually radial, like a bulb, but a reasonable approximation is to say that the sunrays are parallel at the surface of the earth.

# The drone can be added as a light-source (since it emits thermal light) as well
# The sun can be a light-source, and a boolean with "is_sun" or "is_directional" instead of radial check

class Sun():
    def __init__(self, lat, lon):
        self.date_now = datetime.datetime.now(tz=pytz.UTC) # Instead of now, do a slider with +- 12 hours from now
        self.distance_earth_sun = self.calculate_sun_earth_distance(self.date_now)

        # latitude and longditude of HCA Airport test-site shed/hut
        self.lat = lat
        self.lon = lon

        self.altitude = get_altitude(self.lat, self.lon, self.date_now, 13.0)
        self.azimuth = get_azimuth(self.lat, self.lon, self.date_now, 13.0)


    def calculate_sun_earth_distance(self, when):
        '''Returns the distance between the Earth and Sun in meters'''
        jde = stime.get_julian_ephemeris_day(when)
        jce = stime.get_julian_ephemeris_century(jde)
        jme = stime.get_julian_ephemeris_millennium(jce)
        sun_earth_distance_AU = get_sun_earth_distance(jme)
        sun_earth_distance_M = sun_earth_distance_AU*149597870700.0

        return sun_earth_distance_M

    def get_lightsource(self):
        s = mcolors.LightSource(azdeg = self.azimuth, altdeg = self.altitude)
        return s

    def cast_on(self, surfaces, axes):
        # We know the position of the sun in degrees from north (0 to 360) and degrees of altitude (from -90 to 90 I think)


        # TODO: check for surfaces before it hits the window

        # TODO: if there is a surface halfway through the window, get the vertices that are inside the window, not just the edges

        # Create soa in format: [X, Y, Z, U, V, W] where xyz is origin point of vector and uvw is vector
        xyz_end = []
        end_index = len(surfaces[0].x) - 1
        soa = []
        for i in range(len(surfaces)): 
            # First we define the end-points of the vectors"
            xyz_end.append([surfaces[i].x[0], surfaces[i].y[0], surfaces[i].z[0]])
            xyz_end.append([surfaces[i].x[end_index], surfaces[i].y[end_index], surfaces[i].z[0]])
            xyz_end.append([surfaces[i].x[0], surfaces[i].y[0], surfaces[i].z[end_index]])
            xyz_end.append([surfaces[i].x[end_index], surfaces[i].y[end_index], surfaces[i].z[end_index]])

            # Project the sunrays backwards from the endpoints towards the direction of the sun
            r, theta, phi = 5.0, (self.azimuth) * math.pi/180.0, (90 - self.altitude) * math.pi/180.0

            for j in range(len(xyz_end)):
                x, y, z = xyz_end[j][0] + r*math.sin(theta)*math.cos(phi), xyz_end[j][1] + r*math.sin(theta)*math.sin(phi), xyz_end[j][2] + r*math.cos(phi)
                print(x, y, z)
                soa.append([x, y, z, xyz_end[j][0] - x, xyz_end[j][1] - y, xyz_end[j][2] - z])


        X, Y, Z, U, V, W = zip(*soa)
        axes.quiver(X, Y, Z, U, V, W)



        

