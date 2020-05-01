
#from overpass import API
from time import sleep
from sys import exc_info
import numpy as np
import math
import overpy
from haversine import haversine, Unit

# mean earth radius - https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
_AVG_EARTH_RADIUS_KM = 6371.0088
_AVG_EARTH_RADIUS_M = _AVG_EARTH_RADIUS_KM * 1000

class PolygonExtractor():
    def __init__(self, coordinates, radius):

        self.location = coordinates
        # It's actually not a radius, but a bounding box. Making a bounding box that is broader by the meters asked:
        # https://en.wikipedia.org/wiki/Decimal_degrees
        self.radius = radius

        self.north_east_point = self.coordinate_for_startpoint_and_bearing(self.location, 45, self.radius)
        self.south_west_point = self.coordinate_for_startpoint_and_bearing(self.location, 225, self.radius)
        # Now we have two points that span a bounding box with the drone-coordinate in the middle


        # Find the closest house to these coordinates
        self.api = overpy.Overpass()



    def coordinate_for_startpoint_and_bearing(self, coordinate, bearing, distance):
        '''
        JavaScript: (all angles in radians)
        const φ2 = Math.asin( Math.sin(φ1)*Math.cos(d/R) +
                            Math.cos(φ1)*Math.sin(d/R)*Math.cos(brng) );
        const λ2 = λ1 + Math.atan2(Math.sin(brng)*Math.sin(d/R)*Math.cos(φ1),
                                Math.cos(d/R)-Math.sin(φ1)*Math.sin(φ2));
        '''
        # Now we just smash it into Python instead

        d = distance / math.sin(math.radians(45)) # Depending on whether we want the radius to span to the corners or the edges, we either divide or not with 0.7071 (sin45)
        R = _AVG_EARTH_RADIUS_M
        ang = d/R # Angular distance

        # converting the bearing to radians
        b = math.radians(bearing)
        lat1 = math.radians(coordinate[0])
        lon1 = math.radians(coordinate[1])
        
        lat2 = math.asin(math.sin(lat1) * math.cos(ang) + math.cos(lat1) * math.sin(ang) * math.cos(b))
        lon2 = lon1 + math.atan2(math.sin(b)*math.sin(ang)*math.cos(lat1), math.cos(ang) - math.sin(lat1) * math.sin(lat2))

        return np.array([math.degrees(lat2), math.degrees(lon2)])


    def submit_query_to_overpass(self):
        
        #locations=[(50.406029,30.619727),(27.988056, 86.925278)]

        #https://python-overpy.readthedocs.io/en/latest/example.html

        xx1,yy1=self.south_west_point[0],self.south_west_point[1]
        xx2,yy2=self.north_east_point[0],self.north_east_point[1]
        try:
            result = self.api.query(f"""
                way({xx1},{yy1},{xx2},{yy2}) ["building"];
                (._;>;);
                out body;
                """)

            for way in result.ways:
                print("Name: %s" % way.tags.get("name", "n/a"))
                print("  Building: %s" % way.tags.get("building", "n/a"))
                print("  Nodes: " + str(len(way.nodes)))
                #for node in way.nodes:
                #    print("    Lat: %f, Lon: %f" % (node.lat, node.lon))
            return result
                

        except:
            print(f"Got error: {exc_info()}")
            return None

    def find_closest_way(self, result):
        closest_node = []
        closest_way = []
        distance = self.radius*10 # Just something that is way more than the radius of the current bounding box.
        for way in result.ways:
            for node in way.nodes:
                compare_to_this_distance = haversine((self.location[0],self.location[1]),(node.lat,node.lon), unit=Unit.METERS)
                if distance > compare_to_this_distance:
                    closest_node = node
                    closest_way = way
                    distance = compare_to_this_distance
        
        return closest_way, closest_node

    def get_closest_building(self):

        result = self.submit_query_to_overpass()

        closest_way, closest_node = self.find_closest_way(result)
        distances_between_nodes = []
        try:
            for i in range(len(closest_way.nodes) - 1):
                distances_between_nodes.append(haversine((closest_way.nodes[i].lat, closest_way.nodes[i].lon), (closest_way.nodes[i+1].lat, closest_way.nodes[i+1].lon), unit=Unit.METERS))
        except:
            rospy.loginfo("There are no houses nearby.")
        #print(distances_between_nodes)

        return closest_way, closest_node, distances_between_nodes

    

#PE = PolygonExtractor(np.array([55.3729781, 10.4008157]), 20)
#PE.get_closest_building()