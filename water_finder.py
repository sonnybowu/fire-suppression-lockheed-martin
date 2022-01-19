import json
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
'''
f = open('../maps/boston_fire.json')
data = json.load(f)

# (xmax,ymax,xmin,ymin)
corner_coords = [x[0] for x in data["bounds"].values()]
points = [Point(corner_coords[3],corner_coords[2]), Point(corner_coords[3],corner_coords[0]),Point(corner_coords[1],corner_coords[0]),Point(corner_coords[1], corner_coords[2])]


f.close()

#
l = []
g = open('../data/waterbodies.geojson')

data2 = json.load(g)

for feature in data2["features"]:
    for coord in feature['geometry']['coordinates']:
        if (coord[0] > corner_coords[2]) and (coord[0] < corner_coords[0]) and (coord[1] > corner_coords[3]) and (coord[1] < corner_coords[1]):
            l.append(feature["properties"]["name"])
            break
print(l)

print(data2["features"][1]['geometry']['coordinates'][0][0])
print(corner_coords[0])


for feature in data2["features"]:
    for i in points:
        if i.within(Polygon(feature['geometry']['coordinates'])):
            l.append(feature['properties']['name'])
            break
print(l)

g.close()
'''

def waterCoords(l, n):
    '''
    l: list of coordinates of polygon from water in [(x,y), (x,y)] format. Assumes len(l) >= 2
    n: number of coordinates wanted between each adjacent coordinate
    Returns a list of coordinates g given from adjacent coordinates in l with n subdivisions
    '''
    g = []
    for i in range(len(l) - 1):
        delx = l[i+1][0] - l[i][0]
        m = (l[i+1][1] - l[i][1]) / delx
        j = 0
        while j <= n:
            x = l[i][0] + (delx/(n+1))*j
            y = m*(x-l[i][0]) + l[i][1]
            g.append((x,y))
            j += 1
    g.append(l[-1])
    return g

l = [(0,0),(1,5),(2,6),(1,5)]
n = 2
print(waterCoords(l,n))
        