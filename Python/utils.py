#
# Utility functions: file reading, organizing, string processing, etc.
#

from shapes import *


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
    points = [np[1] for np in sorted(numbered_points, key=lambda x:x[0])]

    return Poly(points)
