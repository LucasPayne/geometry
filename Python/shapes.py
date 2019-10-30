"""
    Shape class definitions and basic constructions
"""

import sys
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
def normalized(vector):
    return vector * (1/norm(vector))
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
    def __repr__(self):
        return f"[{self.a}-{self.b}]"


class Line(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def centroid(self):
        return None
    def __repr__(self):
        return f"[{self.a}---{self.b}]"


class Ray(Shape):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    def centroid(self):
        return None
    def __repr__(self):
        return f"[{self.a}-->{self.b}]"


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
    def __repr__(self):
        return f"[{self.A}-{self.B}-{self.C}-.]"


class AABB(Shape):
    """ representation: origin and non-negative ->right extent, ^up extent
        Other possible representations are radial from center and bottom-left-top-right etc.
    """
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
    def __repr__(self):
        return f"AABB({self.origin},\n\t->{self.horiz},\n\t^{self.vert})"


class OBB(Shape):
    def __init__(self, origin, right_dir, horiz, vert):
        self.right = right_dir * (1/norm(right_dir))
        self.up = perp(self.right)
        self.origin = origin
        self.horiz = horiz
        self.vert = vert

    def top_right_point(self):
        return self.origin + self.horiz * self.right + self.vert * self.up
    def top_point(self):
        return self.origin + self.horiz * self.right + self.vert * self.up
    def right_point(self):
        return self.origin + self.horiz * self.right + self.vert * self.up

    def points(self):
        return [self.origin, self.origin + self.horiz * self.right,
                             self.origin + self.horiz * self.right + self.vert * self.up,
                             self.origin + self.vert * self.up]
    def segments(self):
        points = self.points()
        return [LineSeg(points[i], points[(i + 1)%len(points)]) for i in range(len(points))]
    def centroid(self):
        return self.origin + (self.horiz / 2) * self.right + (self.vert / 2) * self.up
    def __repr__(self):
        return f"OBB({self.origin},\n\t{self.right}->{self.horiz},\n\t{self.up}^{self.vert})"


class Graph:
    def incident_nodes(node):
        """ """

class PolygonGraph(Graph):
    """ Bidirectional
        Node: index into polygon
    """
    def __init__(self, poly):
        self.poly = poly
        self.n = len(poly)
    def incident_nodes(self, node):
        return [(node - 1) % self.n, (node + 1) % self.n]

class PolygonHullGraph(Graph):
    """ Bidirectional
        Node: index into polygon
        Technically, there are incident nodes to vertices not on the hull. To work as a graph
        of the hull, as indices into the polygon, the search should be started on the hull.
    """
    def __init__(self, hulled_poly):
        self.poly = hulled_poly
        self.fillings = hulled_poly.fillings

    def incident_nodes(self, node):
        nodes = []
        skipped_left = False
        skipped_right = False
        for filling in self.fillings:
            if node == filling[1]:
                nodes.append(filling[0])
                skipped_left = True
            elif node == filling[0]:
                nodes.append(filling[1])
                skipped_right = True
        if not skipped_left:
            nodes.append((node - 1) % self.n)
        if not skipped_right:
            nodes.append((node + 1) % self.n)


        
class Poly(Shape):
    def __init__(self, points):
        self.points = points
        self.graph = PolygonGraph(self)
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
    def segments(self):
        return [LineSeg(self.points[i], self.points[(i + 1)%len(self.points)]) for i in range(len(self.points))]

    def centroid(self):
        triangles = triangulation_triangles(self)
        return barycentric_to_cartesian([t.centroid() for t in triangles], [t.area() for t in triangles])

    def __add__(self, point):
        if type(point) is Point:
            return Poly([p + point for p in self.points])
        else:
            return None



class Polygon(metaclass=ABCMeta):
    @abstractmethod
    def traversal(self, start_node):
        """ """

class HulledPoly(Poly):
    """
        Acts as the normal Poly, but has extra information about a hull polygon, which
        is itself a Poly, which can be accessed and traversed.
    """

    class Hull(Poly):
        def __init__(self, parent_poly):
            self.parent_poly = parent_poly

    def __init__(self, points):
        super().__init__(points)
        self.fillings, self.hull_len = convex_hull_poly(self)
    
    def hull(self):
        """ Temporary implementation.
            The point of the fillings representation should be that they mask the graph of the polygon,
            and algorithms should not have random access into the polygon indices, rather a "generator",
            an implicit graph interface, will allow algorithms to work with the convex hull as a polygon,
            memory overhead only in the (probably) small number of fillings.

            How should this masking be done nicely? Similarly, convex partitions, triangulations, etc, should
            be traversable as shapes, yet only stored "masked" onto the polygon.

            --- Replace with traversal
        """

        start_index = min(enumerate(self.points), key=lambda ip: (ip[1].x, ip[1].y))[0]
        indices = [start_index]
        i = start_index
        while True:
            skipped = False
            for filling in self.fillings:
                if filling[0] == i:
                    i = filling[1]
                    skipped = True
                    break
            if not skipped:
                i = (i + 1) % len(self)
            if i == start_index:
                break
            indices.append(i)
            
        return indices

    # Generator: polygon traversal
    # A generator is constructed from a 
    
    def traversal(self, start_index):
        """ Does not stop. If not on the hull, this "revs" it up to the hull, and
            continually traverses it.

            This formulation as a generator is only possible because the polygon forms a simple closed curve, meaning its graph has a natural unambiguous traversal.
            For example, far more complicated, if a convex hull of a polyhedra is stored as filling polygons, then for traversal algorithms will just work
            with the graph, which will have incident nodes connected taking into consideration the filling polygons.
        """
        yield start_index
        graph = PolygonHullGraph(self)
        index = start_index
        last_index = start_index




class Circle(Shape):
    def __init__(self, point, radius):
        self.point = point
        self.radius = radius
    def centroid(self):
        return self.point

############   Coordinates    #####################################

class CartesianFrame:
    # Orthonormal coordinate system
    def __init__(self, origin, e1, e2):
        self.origin = origin
        self.e1 = e1
        self.e2 = e2
        self.orthogonalize_e1()
        self.normalize()

    def orthogonalize_e2(self):
        self.e2 = self.e2 - dot(self.e2, normalized(self.e1)) * normalized(self.e1)
    def orthogonalize_e1(self):
        self.e1 = self.e1 - dot(self.e1, normalized(self.e2)) * normalized(self.e2)

    def normalize(self):
        self.e1 = normalized(self.e1)
        self.e2 = normalized(self.e2)

    @staticmethod
    def ambient():
        return CartesianFrame(Point(0, 0), Point(1, 0), Point(0, 1))


###################################################################

def minkowski(locus, moving):
    if type(locus) is Triangle and type(moving) is Circle:
        # >> add the rectangles
        triangle = locus
        circle = moving
        return [Circle(triangle.A, circle.radius),
                Circle(triangle.B, circle.radius),
                Circle(triangle.C, circle.radius)]


# >>>
def transformed(obj, frame):
    T = (type(obj), type(frame))

    if T == (OBB, CartesianFrame):
        box = obj

        new_origin = Point(dot(box.origin - frame.origin, normalized(frame.e1)),
                           dot(box.origin - frame.origin, normalized(frame.e2)))

        new_right = Point(dot(box.right, frame.e1),
                          dot(box.right, frame.e2))

        obb = OBB(new_origin, new_right, box.horiz, box.vert)
        # the auto (-y, x) perp up-direction seems to be wrong for this case (in reflecting coordinates)
        obb.up = Point(dot(box.up, frame.e1),
                       dot(box.up, frame.e2))
        return obb

    else:
        print(f"Error: no transformation defined for {T}")
        sys.exit()

