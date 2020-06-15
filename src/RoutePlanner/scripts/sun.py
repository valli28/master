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
        self.date = datetime.datetime.now(tz=pytz.timezone("Europe/Copenhagen")) # Instead of now, do a slider with +- 12 hours from now
        self.date = datetime.datetime(2020, 1, 15, 9, 0, 0, 0, tzinfo=pytz.timezone("Europe/Copenhagen"))

        self.distance_earth_sun = self.calculate_sun_earth_distance(self.date)

        # latitude and longditude of HCA Airport test-site shed/hut
        self.lat = lat
        self.lon = lon

        self.altitude = get_altitude(self.lat, self.lon, self.date, elevation=13.0)
        self.azimuth = get_azimuth(self.lat, self.lon, self.date, elevation=13.0)

        self.vectors = []
        self.r_vectors = []

        self.roa = []


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
        # The cast_on method gets called every time we update the time we wish to plot things for, so we update the azimuth and altitude.
        self.altitude = get_altitude(self.lat, self.lon, self.date, elevation=13.0)
        self.azimuth = get_azimuth(self.lat, self.lon, self.date, elevation=13.0)

        # TODO: if there is a surface halfway through the window, get the vertices that are inside the window, not just the edges
        #print(surfaces)
        # Create soa in format: [X, Y, Z, U, V, W] where xyz is origin point of vector and uvw is vector
        xyz_end = []

        
        soa = [] # Sun-vectors
        self.roa = [] # Reflection vectors
        if surfaces != []:
            for i in range(len(surfaces)): 
                if len(surfaces[i].x) != 0:
                    end_index_x = len(surfaces[i].x) - 1
                    end_index_y = len(surfaces[i].y) - 1
                    end_index_z = len(surfaces[i].z) - 1
                    # First we define the end-points of the vectors"
                    xyz_end.append([surfaces[i].x[0], surfaces[i].y[0], surfaces[i].z[0]])
                    xyz_end.append([surfaces[i].x[end_index_x], surfaces[i].y[end_index_y], surfaces[i].z[0]])
                    xyz_end.append([surfaces[i].x[0], surfaces[i].y[0], surfaces[i].z[end_index_z]])
                    xyz_end.append([surfaces[i].x[end_index_x], surfaces[i].y[end_index_y], surfaces[i].z[end_index_z]])

                    # Project the sunrays backwards from the endpoints towards the direction of the sun
                    
                    r, theta, phi = 1, (90 - self.azimuth) * math.pi/180.0, (90 - self.altitude) * math.pi/180.0
                    
                    # We can increase the performance of this function by NOT calculating everything 4 times when I actually only have to do it once. 
                    for j in range(i*4, len(xyz_end)):
                        x, y, z = xyz_end[j][0] + r*math.sin(phi)*math.cos(theta), xyz_end[j][1] + r*math.sin(theta)*math.sin(phi), xyz_end[j][2] + r*math.cos(phi)
                        u, v, w = xyz_end[j][0] - x, xyz_end[j][1] - y, xyz_end[j][2] - z

                        [u, v, w] = [u, v, w] / np.linalg.norm([u, v, w])

                        # We check the entrance angle to the plane.
                        incidence_angle = np.arccos(np.clip(np.dot([u, v, w], surfaces[i].normal), -1.0, 1.0))
                        specular_vector = 2*np.dot(surfaces[i].normal, [u, v, w])* surfaces[i].normal - [u, v, w]
                                        

                        # We check whether the sunray goes through a wall first by cheating a little bit. 
                        # The way we do it is by finding whether the normal of the plane is pointing in the opposite direction of the sun.
                        # We use the dot-product to see how the vectors are pointing with respect to eachother. If the dot-product is negative, they are pointing in opposite directions.
                        dot_product = np.dot(surfaces[i].normal, [u, v, w])
                        ground_product = np.dot([0, 0, 1], [u, v, w])
                        if (dot_product > 0 and ground_product < 0):
                            soa.append([x, y, z, u, v, w])
                            self.roa.append([xyz_end[j][0], xyz_end[j][1], xyz_end[j][2], -specular_vector[0], -specular_vector[1], -specular_vector[2]])
                        else: 
                            soa.append([0, 0, 0, 0, 0, 0])
                            self.roa.append([0, 0, 0, 0, 0, 0])

                
        Xr, Yr, Zr, Ur, Vr, Wr = zip(*self.roa)
        X, Y, Z, U, V, W = zip(*soa)

        if self.vectors != []:
            self.vectors.remove()
            self.r_vectors.remove()
        self.vectors = axes.quiver(X, Y, Z, U, V, W, color='r')
        self.r_vectors = axes.quiver(Xr, Yr, Zr, Ur, Vr, Wr)



        

