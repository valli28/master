import numpy as np
import matplotlib.pyplot as plt
import math
import overpy
from haversine import haversine, Unit
import utm


from shapely.geometry import Polygon, LinearRing, Point

from progress.bar import Bar


with open("interpreter_isolated", 'r') as f:
    data = f.read()

ov = overpy.Overpass()
result = ov.parse_xml(data)

n_ways = len(result.ways)
print("Ways: " + str(n_ways))
bar = Bar('Finding isolated buildings', max = n_ways*n_ways)

#for element in result:

#print(data)
#print("Name: %s" % way.tags.get("name", "n/a"))

def check_if_isolated(curr, comp):
    for k in range(len(curr.nodes)):
        for l in range(len(comp.nodes)):
            distance = haversine((curr.nodes[k].lat, curr.nodes[k].lon), (comp.nodes[l].lat, comp.nodes[l].lon), unit=Unit.METERS)

            if distance < 5.0:
                print("found building within 5 meters.")
                return

    return curr

isolated_buildings = []

for i in range(len(result.ways)):
    for j in range(len(result.ways)):
        if i != j:
            current_way = result.ways[i]
            compare_way = result.ways[j]

            

            # Instead of comparing ALL nodes in all of odense together twice, we will find all buildings within 1 km centroid
            dist = haversine((current_way.nodes[0].lat, current_way.nodes[0].lon), (compare_way.nodes[0].lat, compare_way.nodes[0].lon), unit=Unit.METERS)
            if dist < 250.0:
                isolated_buildings.append(check_if_isolated(current_way, compare_way))
                bar.next()


bar.finish()



prin("Isolated ways: " + str(len(isolated_buildings)))