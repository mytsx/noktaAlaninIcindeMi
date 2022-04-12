
import math


def centerOfPoints(coord):
    xList = []
    yList = []
    for c in coord:
        xList.append(c[0])
        yList.append(c[1])
    center = sum(xList)/len(xList), sum(yList)/len(yList)
    return center

def clockwiseangle_and_distance(point):
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector

pts = [
        [653298, 4201883],
        [653340, 4201260],
        [653459, 4201840],
        [653460, 4201440],
        [653500, 4201440],
        [653500, 4201245],
        [653225, 4201750]
        ]

refvec = [1, 0]
origin = centerOfPoints(pts)
print(sorted(pts, key=clockwiseangle_and_distance))
