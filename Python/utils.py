"""
    Utility functions: file reading, organizing, string processing, etc.
"""

from shapes import *
import os
import sys
import functools

def make_poly_from_text(filename):
    lines = open(filename).read().splitlines()
    horiz = len(lines[0])
    vert = len(lines)

    numbered_points = []

    for i in range(vert):
        for j in range(horiz):
            val = lines[i][j]
            if val.isdigit() or val in ['A', 'B', 'C', 'D', 'E', 'F']:
                numbered_points.append((int(val, 16), Point(j/horiz, 1 - i/vert)))

    # print(numbered_points)
    # Make sure polygons are in the right order, or you will end up randomly changing orienting code
    points = [np[1] for np in sorted(numbered_points, key=lambda x:x[0])]

    return Poly(points)


def data_polys():
    """ Generates the polygons from the data/ directory """
    return [make_poly_from_text(f"data/{filename}") for filename in os.listdir("data") if filename.endswith(".poly")]


def prefix_function(function, prefunction):
    # from SO: hook python module function
    @functools.wraps(function)
    def run(*args, **kwargs):
        prefunction(*args, **kwargs)
        return function(*args, **kwargs)
    return run

