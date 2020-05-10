
#from overpass import API
from time import sleep
from sys import exc_info
import numpy as np
import matplotlib.pyplot as plt
import math
import overpy
from haversine import haversine, Unit
import utm
from mission import Mission
from offb.msg import Bool, BuildingPolygonResult
from geometry_msgs.msg import Pose, PoseArray
from shapely.geometry import Polygon, LinearRing

# mean earth radius - https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
_AVG_EARTH_RADIUS_KM = 6371.0088
_AVG_EARTH_RADIUS_M = _AVG_EARTH_RADIUS_KM * 1000


class PolygonExtractor():
    def __init__(self, coordinates, origin, radius):

        self.origin = origin
        self.location = coordinates
        # It's actually not a radius, but a bounding box. Making a bounding box that is broader by the meters asked:
        # https://en.wikipedia.org/wiki/Decimal_degrees
        self.radius = radius

        self.north_east_point = self.coordinate_for_startpoint_and_bearing(self.location, 45, self.radius)
        self.south_west_point = self.coordinate_for_startpoint_and_bearing(self.location, 225, self.radius)
        # Now we have two points that span a bounding box with the drone-coordinate in the middle


        # Find the closest house to these coordinates
        self.api = overpy.Overpass()

        self.house_nodes_positions = []
        self.house_impression_positions = []
        self.house_corner_positions = []
        self.house_impression_direction = []
        self.building_height = 2 # TODO: Take input from pilot
        self.wall_lengths = []
        self.overpass_building = []
        


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
        

        print("Name: %s" % closest_way.tags.get("name", "n/a"))
        print("  Building: %s" % closest_way.tags.get("building", "n/a"))
        print("  Approximate height %s" % closest_way.tags.get("building:height", "n/a"))
        print("  Nodes: " + str(len(closest_way.nodes)))
        for node in closest_way.nodes:
            print("    Lat: %f, Lon: %f" % (node.lat, node.lon))
            print("    And my position is currently Lat: %f, Lon: %f" % (self.location[0], self.location[1]))

        #print("Which in local coordinates, with origin as the origin, that corresponds to:")
        x, y, number, letter = utm.from_latlon(self.origin[0], self.origin[1])
        local_coords = []
        for node in closest_way.nodes:
            pointlat = node.lat
            pointlon = node.lon
            pointx, pointy, number, letter = utm.from_latlon(float(pointlat), float(pointlon))
            difx = x - pointx
            dify = y - pointy
            
            #print("  x: %f, y: %f" % (difx, dify))
            local_coords.append(np.array([-difx, -dify]))

        # Simplify local coordinates so that we don't get multiple vertices on essentialy a single edge
        shapely_polygon = LinearRing(local_coords)
        simplified_polygon = shapely_polygon.simplify(0.8)
        px, py = simplified_polygon.coords.xy
        # Pack these simplified values into the local_coords
        local_coords = []
        for i in range(len(px)):
            local_coords.append(np.array([px[i], py[i]]))

        # And recalculate the distance between the existing nodes
        self.wall_lengths = []
        for i in range(len(local_coords) - 1):
            dist = math.sqrt((local_coords[i][0] - local_coords[i+1][0])**2 + (local_coords[i][1] - local_coords[i+1][1])**2)
            self.wall_lengths.append(dist)

        # We use Shapely's parallel_offset function to create a list of positions that are offset from the walls by some constant
        # These positions will make sure that the drone wont crash into the corners of the buildings
        self.house_corner_positions = simplified_polygon.parallel_offset(5.0, 'left', join_style=2) # TODO: some distance that is actually intelligent

        self.house_nodes_positions = local_coords
        
        return closest_way, closest_node, local_coords

    def get_closest_building(self):

        result = self.submit_query_to_overpass()

        closest_way, closest_node, local_coords = self.find_closest_way(result)
        #distances_between_nodes = []
        building_height = []
        try:
            #for i in range(len(closest_way.nodes) - 1):
                #distances_between_nodes.append(haversine((closest_way.nodes[i].lat, closest_way.nodes[i].lon), (closest_way.nodes[i+1].lat, closest_way.nodes[i+1].lon), unit=Unit.METERS))
            
            building_height = closest_way.tags.get("building:height", "n/a")

        except:
            print("There are no houses nearby.")
        
        try:
            self.building_height = int(building_height)
        except:
            print("Building does not have a registered height. Going back to default of " + str(self.building_height) + "m")

        self.overpass_building = closest_way
        #self.wall_lengths = distances_between_nodes

        return closest_way, closest_node, building_height, local_coords
        

    def generate_impression_poses(self):
        # and remember that the nodes in the house variable and the distances are ordered correctly corresponding to the order which they are connected.
        print("I'm in the generate impression poses function now")

        building = BuildingPolygonResult()
        ''' BuildingPolygonResult.msg
        bool found_building
        string building_name
        string building_type
        float64[] distances
        '''
        building.found_building = True
        building.building_name = self.overpass_building.tags.get("name", "n/a")
        building.building_type = self.overpass_building.tags.get("building", "n/a")
        building.distances = self.wall_lengths

        poses = PoseArray()
        #poses.header.stamp = 0 # TODO: Time
        poses.header.frame_id = "impression_poses"

        mission = Mission(80, 75)

        poses_list = []
        for i in range(len(self.wall_lengths)):
            pose = Pose()
            
            # since we know about the parameters of the camera, and we don't care about how many pixels or how much GSD we want, we can just do a triangle calculation I guess...
            # Lets get the horizontal distance first
            h = abs((self.wall_lengths[i] / 2.0) * math.tan((180 - mission.aov[0])))
            # And then the vertical
            v = abs((self.building_height / 2.0) * math.tan((180 - mission.aov[1])))
            # And compare which one of them is largest and return it
            if h > v:
                distance_from_wall = h
            else:
                print("Building is too high. Backing off a bit.")
                distance_from_wall = v

            # Now we have everything we need to generate the point at which the drone has to be to look at the wall.
            # We take the coordinates of the endpoints of the walls

            point_a = self.house_nodes_positions[i]
            point_b = self.house_nodes_positions[i+1]
            

            # TODO: How do we make sure that we don't make a point inside the building?
            # It seems that the nodes are always defined clockwise, which means that if we draw the vector from a to b, the impression_point should be to the "left"
            # if we are looking in the same direction as the a-b vector

            vector_ab = point_b - point_a

            # We half that vector since we are doing that kind of triangle-calculation
            vector_ab *= 0.5

            # since that vector now points to the midpoint between a and b, we create a vector from that point to b and call it bh.
            point_h = point_a + vector_ab

            vector_bh = point_h + vector_ab

            # We rotate this vector anti-clockwise (so by a positive rotation-angle theta)
            # The angle is 180-fov divided by two
            theta = 90
            theta = math.radians(theta)
            # We use the rotation-matrix to rotate the vector 
            vector_hc = np.array([math.cos(theta)* vector_ab[0] - math.sin(theta)*vector_ab[1], math.sin(theta)*vector_ab[0] + math.cos(theta)*vector_ab[1]])
            
            # This vector is sadly not the correct size since it is as long as vector_ab. 
            # We have to enlarge it by multiplying it with a scalar corresponding to the ratio between the sides of the triangle and half the wall
            # The length of the arms on the triangle is calculated by its baseline and height, which we know
            #a = math.sqrt((0.5 * self.wall_lengths[i])**2 + distance_from_wall**2)
            #print(a)
            #print(distance_from_wall)
            #print(self.wall_lengths[i])

            # Now we have what is supposed to be how long the vector is. 
            #vector_ac *= a / (self.wall_lengths[i]*0.5)

            # If we add vector_ac to point a, we now have point c, which is the impression position.
            #impression_position_xy = point_a + vector_ac
            impression_position_xy = point_h + (vector_hc / np.linalg.norm(vector_hc)) * (distance_from_wall + 1.5) # Add 1.5m to the desired distance
            # Generate a position that is halfway between the two nodes, as well as a distance away form the facade (like a triangle)
            '''
            This is stupid... but I have to correct the launch offset in xy that I have set the drone to for all the positions that i post to the drone...
                    <arg name="x" value="-35"/>
                    <arg name="y" value="-20"/>
                These are the current values.

            '''

            # TODO: Insert the corner positions into every other pose :) 
            pose.position.x = impression_position_xy[0] + 35
            pose.position.y = impression_position_xy[1] + 20
            pose.position.z = int(self.building_height) / 2 #TODO The height must be something based on the angle of the camera

            self.house_impression_positions.append(impression_position_xy)

            theta = math.radians(-90)
            heading = np.array([math.cos(theta)* vector_ab[0] - math.sin(theta)*vector_ab[1], math.sin(theta)*vector_ab[0] + math.cos(theta)*vector_ab[1]])
            self.house_impression_direction.append(heading / (math.sqrt(heading[0]**2 + heading[1]**2)))
            heading = math.atan(heading[1]/heading[0])
            pose.orientation.z = heading
            
            #string = "Point " + str(i) + " xy: (" + str(pose.position.x) + ", " + str(pose.position.y) + str(")")
            #print(string)
            poses_list.append(pose)

        poses.poses = poses_list

        return building, poses

    def draw(self):
        # unpack all the values in x and y arrays
        x = []
        y = []
        for i in range(len(self.house_nodes_positions)):
            x.append(self.house_nodes_positions[i][0])
            y.append(self.house_nodes_positions[i][1])

        fig, ax = plt.subplots(1, 1)

        plt.plot(x, y, marker='o', color='b')
        ax.axis('equal')
        
        # And the impression poses in a slightly different color
        xi = []
        yi = []
        u = []
        v = []
        for i in range(len(self.house_impression_positions)):
            xi.append(self.house_impression_positions[i][0])
            yi.append(self.house_impression_positions[i][1])
            u.append(self.house_impression_direction[i][0])
            v.append(self.house_impression_direction[i][1])
            
        #plt.scatter(xi, yi, marker='o', color='r')
        plt.quiver(xi, yi, u, v, color='r')

        xxx, yyy = self.house_corner_positions.coords.xy
        plt.scatter(xxx, yyy)

        plt.show()



spawn = np.array([55.396142, 10.388953])
origin = np.array([55.396142, 10.388953])
PE = PolygonExtractor(spawn, origin, 20) # Coordinates a meter away from building
PE.get_closest_building()

PE.generate_impression_poses()

PE.draw()
