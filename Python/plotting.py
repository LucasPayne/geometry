#
# Plotting functions
#

import sys
from shapes import *
from utils import *
from operations import *
from intersections import *

from matplotlib import pyplot as plt

def plot(obj, **kwargs):
    T = type(obj)

    if T == LineSeg:
        plt.plot([obj.a.x, obj.b.x], [obj.a.y, obj.b.y], **kwargs)

    elif T == Ray:
        plot(LineSeg(obj.a, obj.b), **kwargs)
        plot_arrowhead(obj.b, obj.b - obj.a)

    elif T == Line:
        mid = midpoint(obj.a, obj.b)
        plot(LineSeg(mid + 5.0 * (obj.a - mid), mid + 5.0 * (obj.b - mid)), color='lime')
        plot(LineSeg(obj.a, obj.b), **kwargs)

    elif T == Poly:
        if len(obj) > 0:
            plt.plot([p.x for p in obj] + [obj[0].x], [p.y for p in obj] + [obj[0].y], **kwargs)

    elif T == Point:
        plt.scatter([obj.x], [obj.y], **kwargs)

    elif T == Triangle:
        triangle = obj
        plt.plot([triangle.A.x, triangle.B.x, triangle.C.x, triangle.A.x],
                 [triangle.A.y, triangle.B.y, triangle.C.y, triangle.A.y],
                 **kwargs)

    elif T == Circle:
        plot_circle(obj, **kwargs)

    elif T == AABB:
        plot(obj.origin, **kwargs)
        plot(obj.segments(), **kwargs)

    elif T == OBB:
        plot(obj.origin, **kwargs)
        plot(obj.segments(), **kwargs)


    elif  T == CartesianFrame:
        plot(obj.origin, **kwargs)
        plot(LineSeg(obj.origin, obj.origin + obj.e1), **kwargs)
        plot(LineSeg(obj.origin, obj.origin + obj.e2), **kwargs)
        plot_arrowhead(obj.origin + obj.e1, obj.e1, color='b')
        plot_arrowhead(obj.origin + obj.e2, obj.e2, color='r')
    elif T == list:
        for o in obj:
            plot(o, **kwargs)


    else:
        print(f"Error: plotting not defined for {T}")
        sys.exit()


def plot_circle(circle, **kwargs):
    points_x = []
    points_y = []
    for i in range(360):
        xc = cos(2*pi/360*i)
        yc = sin(2*pi/360*i)
        points_x.append(circle.point.x + circle.radius * xc)
        points_y.append(circle.point.y + circle.radius * yc)
    plt.scatter([circle.point.x], [circle.point.y], **kwargs)
    plt.plot(points_x, points_y, **kwargs)


def plot_arrowhead(point, direction, color='r', alpha=1):
    horiz_n = normal_perp(direction)
    vert_n = direction * (1/norm(direction))
    arrowhead1 = point + 0.03*horiz_n - 0.03*vert_n
    arrowhead2 = point - 0.03*horiz_n - 0.03*vert_n
    plt.plot([arrowhead1.x, point.x, arrowhead2.x],
             [arrowhead1.y, point.y, arrowhead2.y],
             color=color, alpha=alpha)

def plot_hatch_convex_poly(poly, **kwargs):
    minx = min(p.x for p in poly)
    maxx = max(p.x for p in poly)
    
    n = 100
    for i in range(n):
        x = minx + i * (maxx - minx)/n
        line = Line(Point(x, -1), Point(x, 1))


        intersect = intersection(line, poly)
        if intersect and len(intersect) == 2:
            plot(LineSeg(intersect[0], intersect[1]), **kwargs)
