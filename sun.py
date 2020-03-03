import numpy as np
from pysolar.solar import *
import datetime

class Sun():
    def __init__(self, lat, lon):
        self.date_now = datetime.datetime.now(tz=pytz.UTC)
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

    def get_NED_position(self):
        print("")

        

