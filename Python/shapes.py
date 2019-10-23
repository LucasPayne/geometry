#
# Shape class definitions and basic constructions
#

from math import sin, cos, pi, sqrt
from collections import namedtuple
from random import random, randrange
from abc import ABCMeta, abstractmethod 

# >>>>>>>>>>>>>>>>>>>
def norm(vector):
    return sqrt(square_length(vector))
def perp(v):
    return Point(-v.y, v.x)
def normal_perp(v):
    return perp(v) * (1/norm(v))
def square_length(vector):
    return vector.x ** 2 + vector.y ** 2
def det(v1, v2):
    return dot(v1, Point(-v2.y, v2.x))
def dot(v1, v2):
    return v1.x * v2.x + v1.y * v2.y
# >>>>>>>>>>>>>>>>>>>


class Shape(metaclass=ABCMeta):
    @abstractmethod
    def centroid(self):
        """ """


class Point(Shape):
    """ Currently no type delineation between "point" and "vector" """
    @staticmethod
    def random(r):
        return Point(random() * r, random() * r)
    def __repr__(self):
        return f"({self.x},{self.y})"
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)
    def __mul__(self, scalar):
        return Point(self.x * scalar, self.y * scalar)
    def __rmul__(self, scalar):
        return Point(self.x * scalar, self.y * scalar)

    def centroid(self):
        return self


class LineSeg(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def centroid(self):
        return midpoint(self.a, self.b)


class Line(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def centroid(self):
        return None


class Ray(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def centroid(self):
        return None


class Triangle(Shape):
    def __init__(self, A, B, C):
        self.A = A
        self.B = B
        self.C = C
    def area(self):
        return 1/2 * det(self.B - self.A, self.C - self.A)
    def __iter__(self):
        for p in [self.A, self.B, self.C]:
            yield p
    def __len__(self):
        return 3
    def __getitem__(self, key):
        if key == 0:
            return self.A
        if key == 1:
            return self.B
        if key == 2:
            return self.C
        raise KeyError
    def points(self):
        return [self.A, self.B, self.C]
    def centroid(self):
        return barycentric_to_cartesian(self.points, [1, 1, 1])


class AABB(Shape):
    def __init__(self, origin, horiz, vert):
        self.origin = origin
        self.horiz = horiz
        self.vert = vert
    def points(self):
        return [self.origin, self.origin + Point(self.horiz, 0),
                             self.origin + Point(self.horiz, self.vert),
                             self.origin + Point(0, self.vert)]
    def segments(self):
        points = self.points()
        return [LineSeg(points[i], points[(i + 1)%len(points)]) for i in range(len(points))]
    def centroid(self):
        return self.origin + Point(horiz / 2, vert / 2)


class OBB(Shape):
    def __init__(self, origin, right_dir, horiz, vert):
        self.right = right_dir * (1/norm(right_dir))
        self.up = perp(self.right)
        self.origin = origin
        self.horiz = horiz
        self.vert = vert
    def points(self):
        return [self.origin, self.origin + self.horiz * self.right,
                             self.origin + self.horiz * self.right + self.vert * self.up,
                             self.origin + self.vert * self.up]
    def segments(self):
        points = self.points()
        return [LineSeg(points[i], points[(i + 1)%len(points)]) for i in range(len(points))]
    def centroid(self):
        return self.origin + (self.horiz / 2) * self.right + (self.vert / 2) * self.up


class Poly(Shape):
    def __init__(self, points):
        self.points = points
    def lines(self):
        for i, a in enumerate(self.points):
            b = self.points[(i + 1) % len(self.points)]
            yield LineSeg(a, b)
    def __iter__(self):
        for p in self.points:
            yield p
    def __len__(self):
        return len(self.points)
    def __getitem__(self, key):
        return self.points[key]

    def centroid(self):
        triangles = triangulate(self)
        return barycentric_to_cartesian([t.centroid() for t in triangles], [t.area() for t in triangles])


class Circle(Shape):
    def __init__(self, point, radius):
        self.point = point
        self.radius = radius
    def centroid(self):
        return self.point


###################################################################

def minkowski(locus, moving):
    if type(locus) is Triangle and type(moving) is Circle:
        # >> add the rectangles
        triangle = locus
        circle = moving
        return [Circle(triangle.A, circle.radius),
                Circle(triangle.B, circle.radius),
                Circle(triangle.C, circle.radius)]

