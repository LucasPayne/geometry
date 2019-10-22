#
# Vector and coordinate operations
#

from shapes import *


def det(v1, v2):
    return dot(v1, Point(-v2.y, v2.x))


def square_length(vector):
    return vector.x ** 2 + vector.y ** 2


def norm(vector):
    return sqrt(square_length(vector))


def dot(v1, v2):
    return v1.x * v2.x + v1.y * v2.y


def normal_perp(v):
    return Point(-v.y, v.x) * (1/norm(v))


def weighted_midpoint(p1, p2, w1, w2):
    # sum to zero?
    return p1 + (w2 / (w1 + w2)) * (p2 - p1)


def midpoint(p1, p2):
    return weighted_midpoint(p1, p2, 1, 1)


def barycentric_to_cartesian(points, weights):
    # really a linear sum function, as non-convex polygons do not form a barycentric basis for the plane (?)
    w = sum(weights)
    # {{{ Error handling
    if len(points) < 2:
        print("Not enough barycentric points")
        sys.exit()
    if len(points) != len(weights):
        print("There must be the same number of weights as points")
        sys.exit()
    if w == 0:
        print("Invalid barycentric weights, sum to zero")
        sys.exit()
    # }}}
    ts = [t / w  for t in weights]
    
    running_centroid = weighted_midpoint(points[0], points[1], ts[0], ts[1])
    running_weight = ts[0] + ts[1]
    for i in range(2, len(ts)):
        running_centroid = weighted_midpoint(running_centroid, points[i], running_weight, ts[i])
        running_weight += ts[i]
    return running_centroid
        

def centroid(points):
    return barycentric_to_cartesian(points, [1 for p in points])
