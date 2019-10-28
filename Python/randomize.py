#
# Random data generation
#

from shapes import *
from triangulation import *
from random import random

def random_convex_polygon(xmin, xmax, ymin, ymax, n, shift=0):
    points = [Point((xmax - xmin) * random() + xmin, (ymax - ymin) * random() + ymin) for _ in range(n)]
    shifted = Point.random(shift)
    return convex_hull([p + shifted for p in points])

