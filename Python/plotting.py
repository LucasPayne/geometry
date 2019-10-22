#
# Plotting functions
#

from shapes import *
from utils import *
from operations import *
from intersections import *

from matplotlib import pyplot as plt

def plot(obj, color='r'):
    if type(obj) is LineSeg:
        plt.plot([obj.a.x, obj.b.x], [obj.a.y, obj.b.y], color=color)
    elif type(obj) is Poly:
        if len(obj) > 0:
            plt.plot([p.x for p in obj] + [obj[0].x], [p.y for p in obj] + [obj[0].y], color=color)
    elif type(obj) is Point:
        plt.scatter([obj.x], [obj.y], color=color)
    elif type(obj) is Triangle:
        triangle = obj
        plt.plot([triangle.A.x, triangle.B.x, triangle.C.x, triangle.A.x],
                 [triangle.A.y, triangle.B.y, triangle.C.y, triangle.A.y],
                 color=color)
    elif type(obj) is Circle:
        plot_circle(obj, color=color)
    elif type(obj) is AABB:
        plot(obj.segments(), color=color)
    elif type(obj) is list:
        for o in obj:
            plot(o, color)


def plot_circle(circle, color='r'):
    points_x = []
    points_y = []
    for i in range(360):
        xc = cos(2*pi/360*i)
        yc = sin(2*pi/360*i)
        points_x.append(circle.point.x + circle.radius * xc)
        points_y.append(circle.point.y + circle.radius * yc)
    plt.scatter([circle.point.x], [circle.point.y], color=color)
    plt.plot(points_x, points_y, color='r')


def plot_arrowhead(point, direction, color='r'):
    horiz_n = normal_perp(direction)
    vert_n = direction * (1/norm(direction))
    arrowhead1 = point + 0.03*horiz_n - 0.03*vert_n
    arrowhead2 = point - 0.03*horiz_n - 0.03*vert_n
    plt.plot([arrowhead1.x, point.x, arrowhead2.x],
             [arrowhead1.y, point.y, arrowhead2.y],
             color=color)


