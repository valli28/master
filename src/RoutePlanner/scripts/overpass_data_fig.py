import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import math
import overpy
import utm

from shapely.geometry import Polygon, Point
from shapely.strtree import STRtree

from progress.bar import Bar

def generate_impression_poses(polygon, height, aov, clockwise):
    xx, yy = polygon.exterior.coords.xy
    wall_lengths = np.sqrt(np.diff(xx)**2 + np.diff(yy)**2)

    house_nodes_positions = np.array([xx, yy]).transpose()
    house_impression_positions = []
    wall_distances = []
    #wall_distances = []
    for i in range(len(wall_lengths)):
        # since we know about the parameters of the camera, and we don't care about how many pixels or how much GSD we want, we can just do a triangle calculation I guess...
        # Lets get the horizontal distance first
        h = abs((wall_lengths[i] / 2.0) * math.tan((180 - aov[0])))
        # And then the vertical
        v = abs((height / 2.0) * math.tan((180 - aov[1])))
        # And compare which one of them is largest and return it
        if h > v:
            distance_from_wall = h + 1.5
        else:
            distance_from_wall = v + 1.5
        wall_distances.append(distance_from_wall)
        # Now we have everything we need to generate the point at which the drone has to be to look at the wall.
        # We take the coordinates of the endpoints of the walls

        point_a = house_nodes_positions[i]
        point_b = house_nodes_positions[i+1]

        vector_ab = point_b - point_a

        # We half that vector since we are doing that kind of triangle-calculation
        vector_ab *= 0.5

        # since that vector now points to the midpoint between a and b, we create a vector from that point to b and call it bh.
        point_h = point_a + vector_ab

        # We rotate this vector anti-clockwise (so by a positive rotation-angle theta)
        # The angle is 180-fov divided by two
        if clockwise:
            theta = 90
        else:
            theta = -90
        theta = math.radians(theta)
        # We use the rotation-matrix to rotate the vector 
        vector_hc = np.array([math.cos(theta)* vector_ab[0] - math.sin(theta)*vector_ab[1], math.sin(theta)*vector_ab[0] + math.cos(theta)*vector_ab[1]])
    
        impression_position_xy = point_h + (vector_hc / np.linalg.norm(vector_hc)) * (distance_from_wall) # Add 1.5m to the desired distance
        # Generate a position that is halfway between the two nodes, as well as a distance away form the facade (like a triangle)
        p = Point(impression_position_xy[0], impression_position_xy[1])
        house_impression_positions.append(p)

    return house_impression_positions



def check_tree(tr, points):
    for point in points:
        result = tree.query(point)
        if result != []:
            return -1
    return 1

    

def signed_area(pr2):
    """Return the signed area enclosed by a ring using the linear time
    algorithm at http://www.cgafaq.info/wiki/Polygon_Area. A value >= 0
    indicates a counter-clockwise oriented ring."""
    xs, ys = map(list, zip(*pr2))
    xs.append(xs[1])
    ys.append(ys[1])
    return sum(xs[i]*(ys[i+1]-ys[i-1]) for i in range(1, len(pr2)))/2.0

def rotation_dir(pr):
    signedarea = signed_area(pr)
    if signedarea > 0:
        return 0
    elif signedarea < 0:
        return 1
    else:
        return "UNKNOWN"

with open("interpreter_big", 'r') as f:
    data = f.read()

ov = overpy.Overpass()
result = ov.parse_xml(data)

n_ways = len(result.ways)
print("Ways: " + str(n_ways))
bar = Bar('Converting ways into shapely', max = n_ways)

origin = np.array([55.396142, 10.388953])
x, y, number, letter = utm.from_latlon(origin[0], origin[1])

sensor_resolution = np.array([1920, 1080])# pixels
aov = np.array([114.592, 114.592*(sensor_resolution[1]/sensor_resolution[0])])  # deg

points_list = []
polygons = []
for way in result.ways:
    local_coords = []
    for node in way.nodes:
        pointlat = node.lat
        pointlon = node.lon
        pointx, pointy, number, letter = utm.from_latlon(float(pointlat), float(pointlon))
        difx = x - pointx
        dify = y - pointy
        
        #print("  x: %f, y: %f" % (difx, dify))
        local_coords.append(np.array([-difx, -dify]))

    # Simplify local coordinates so that we don't get multiple vertices on essentialy a single edge
    poly = Polygon(local_coords)
    poly = poly.simplify(0.8)
    points_list.append(generate_impression_poses(poly, 2, aov, rotation_dir(local_coords)))
    polygons.append(poly)

    bar.next()
bar.finish()

draw = False
if draw:
    fig, ax = plt.subplots(1, 1)
    ax.axis('equal')
    for points in points_list:
        for point in points:
            plt.scatter(point.x, point.y)
    for poly in polygons:
        xxx, yyy = poly.exterior.coords.xy
        plt.plot(xxx, yyy)
    plt.show()


print("Putting polygons into tree")
tree = STRtree(polygons)

bar2 = Bar("Counting impression positions lists for interferences", max = n_ways)
valid_counter = []
invalid_counter = []
pos = []
#counter2 = 0
ymin, ymax, xmin, xmax = 1000000, -1000000, 1000000, -1000000

for points in points_list:
    valid = check_tree(tree, points)
    if valid == 1:
        valid_counter.append(valid)
        invalid_counter.append(0)
    if valid == -1:
        invalid_counter.append(valid)
        valid_counter.append(0)
    pos.append(np.array([points[0].x, points[0].y]))
    
    for point in points:
        if ymin > point.y:
            ymin = point.y
        if ymax < point.y:
            ymax = point.y
        if xmin > point.x:
            xmin = point.x
        if xmax < point.x:
            xmax = point.x
    #counter2 += check(points, polygons)
    bar2.next()
bar2.finish()

res = 100 #meters
_x = np.arange(xmin, xmax, res)
_y = np.arange(ymin, ymax, res)
_xx, _yy = np.meshgrid(_x, _y)
x, y = _xx.ravel(), _yy.ravel()

#print(invalid_counter)

_z_invalid = np.zeros_like(_xx)
_z_valid = np.zeros_like(_xx)
it = 0
invert = _z_invalid.shape[0]
for p in pos:
    try:
        x_index = np.where(np.logical_and(_x < p[0] + res/2, _x > p[0] - res/2))[0][0]
        y_index = invert - np.where(np.logical_and(_y < p[1] + res/2, _y > p[1] - res/2))[0][0]
        _z_invalid[y_index][x_index] -= invalid_counter[it]
        _z_valid[y_index][x_index] += valid_counter[it]
    except:
        #print("Invalid index. Not inserting into z matrix")
        x_index = None
        y_index = None

    it += 1
    #print(x_index, y_index)


#diff = _z_invalid + _z_valid
total = abs(_z_invalid) + abs(_z_valid)


z_invalid_norm = np.divide(_z_invalid, total)
z_valid_norm = np.divide(_z_valid, total)

#print(z_invalid_norm)
z_invalid_norm = np.where(np.isnan(z_invalid_norm), 0, z_invalid_norm)
z_invalid_norm = np.where(np.isinf(z_invalid_norm), 0, z_invalid_norm)

z_valid_norm = np.where(np.isnan(z_valid_norm), 0, z_valid_norm)
z_valid_norm = np.where(np.isinf(z_valid_norm), 0, z_valid_norm)



# Plot first
fig, ax = plt.subplots(2, 1, figsize=(7,11))
ax[0].axis('equal')

ax[0].set_title("Percentage of buildings with invalid impression poses")
ax[0].set_xlabel("Easting from City Hall [500m]")
ax[0].set_ylabel("Northing from City Hall [500m]")
im_invalid = ax[0].imshow(z_invalid_norm*100, cmap=plt.cm.hot)
fig.colorbar(im_invalid, ax=ax)

# Plot second

ax[1].axis('equal')

ax[1].set_title("Percentage of buildings with valid impression poses")
ax[1].set_xlabel("Easting [500m]")
ax[1].set_ylabel("Northing [500m]")
im_valid = ax[1].imshow(z_valid_norm*100, cmap=plt.cm.hot)
#fig[1].colorbar(im_valid, ax=ax2)

images = [im_invalid, im_valid]
# Find the min and max of all colors for use in setting the color scale.
vmin = min(image.get_array().min() for image in images)
vmax = max(image.get_array().max() for image in images)
norm = colors.Normalize(vmin=vmin, vmax=vmax)
for im in images:
    im.set_norm(norm)


plt.savefig("validityDensityOdense.pdf")
plt.savefig("validityDensityOdense.png")

# Plot the figures
plt.show()

# Print some valuable stuff
print("Total number of buildings: " + str(np.sum(total)))
print("Number of houses that are valid: " + str(np.sum(valid_counter)))
print("Number of houses that are invalid: " + str(abs(np.sum(invalid_counter))))
print("Percentage of houses that this approach can cover: " + str(np.sum(valid_counter) / np.sum(total) * 100))

# TODO fix the scales together so they are eqally bright

# Save the data
np.save('outfile_invalid', _z_invalid)
np.save('outfile_valid', _z_valid)
np.save('outfile_pos', pos)


'''  
with open('outfile_pos.npy', 'rb') as f:
    a = np.load(f)
    
print(a)
'''

''' Results 22 May
Ways: 72986
Converting ways into shapely |################################| 72986/72986
Putting polygons into tree
Done
Counting impression positions lists for interferences |################################| 72986/72986
56647
'''


'''
24 may
Ways: 102827
Converting ways into shapely |################################| 102827/102827
Putting polygons into tree
Counting impression positions lists for interferences |################################| 102827/102827
Invalid index. Not inserting into z matrix
Total number of buildings: 102826.0
Number of houses that are valid: 23563
Number of houses that are invalid: 79264
Percentage of houses that this approach can cover: 22.91541049929006
'''

