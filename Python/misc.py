"""
     Stuff that currently doesn't really fit anywhere
"""

def probability_intersection():
    """ computes approx proportion of line segment
        pairs which intersect given their ends are
        uniformly distributed on a square. """
    num = 100000
    intersecting_count = 0
    for _ in range(num):
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        if intersecting(seg1, seg2):
            intersecting_count += 1
    print(intersecting_count / num)
