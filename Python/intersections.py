#
# Intersection tests
# Intersecting?
# Intersection information: contact manifold, overlap polygon
#   (can include polygon collections from polygon intersections, etc.)
#

import sys
from shapes import *
from operations import *
from plotting import *

def intersecting(objA, objB):
    T = (type(objA), type(objB))

    if T == (Triangle, Circle):
        triangle = objA
        circle = objB
        mink_sum = minkowski(triangle, circle)
        return any([point_in(circle.point, part) for part in mink_sum])

    elif T == (LineSeg, LineSeg):
        a1 = objA.a
        a2 = objA.b
        b1 = objB.a
        b2 = objB.b
        return (((det(b1 - a1, a2 - a1) < 0) != (det(b2 - a1, a2 - a1) <= 0)) and
                ((det(a1 - b1, b2 - b1) < 0) != (det(a2 - b1, b2 - b1) <= 0)))

    elif T == (Point, AABB):
        point, box = objA, objB
        return ((box.origin.x <= point.x <= box.origin.x + box.horiz) and
                (box.origin.y <= point.y <= box.origin.y + box.vert))

    elif T == (Point, OBB):
        point, box = objA, objB
        # transform to coords of obb
        pright = dot(point - box.origin, box.right)
        pup = dot(point - box.origin, box.up)
        return ((0 <= pright <= box.horiz) and
                (0 <= pup <= box.vert))

    elif T == (Point, Triangle):
        point, A, B, C = objA, objB.A, objB.B, objB.C
        return (det(B - A, point - A) < 0 and
                det(C - B, point - B) < 0 and
                det(A - C, point - C) < 0)

    elif T == (Point, Poly):
        return point_in_polygon(objA, objB)

    elif T == (Point, Circle):
        point, circle = objA, objB
        return square_length(point - circle.point) < circle.radius ** 2

    elif T == (AABB, AABB):
        box1, box2 = objA, objB
        # Test for separating axes
        if (box1.origin.x + box1.horiz < box2.origin.x
                or box2.origin.x + box2.horiz < box1.origin.x
                or box1.origin.y + box1.vert < box2.origin.y
                or box2.origin.y + box2.vert < box1.origin.y):
            return False
        return True

    elif T == (OBB, OBB):
        box1, box2 = objA, objB
        # Transform the second box into the coordinates of the first and do an AABB-OBB test
        transformed_box = transformed(box2, CartesianFrame(box1.origin, box1.right, box1.up))
        aabb = AABB(Point(0, 0), box1.horiz, box1.vert)

        return intersecting(aabb, transformed_box)    

    elif T == (AABB, OBB):
        aabb, obb = objA, objB
        # check separating axes normal to the aabb sides by trivial projection
        if (min(p.x for p in obb.points()) > aabb.origin.x + aabb.horiz
                or max(p.x for p in obb.points()) < aabb.origin.x):
            return False
        if (min(p.y for p in obb.points()) > aabb.origin.y + aabb.vert
                or max(p.y for p in obb.points()) < aabb.origin.y):
            return False
        # check separating axes normal to the obb
        if (max(dot(p - obb.origin, obb.right) for p in aabb.points()) < 0
                or min(dot(p - obb.origin, obb.right) for p in aabb.points()) > obb.horiz
                or max(dot(p - obb.origin, obb.up) for p in aabb.points()) < 0
                or min(dot(p - obb.origin, obb.up) for p in aabb.points()) > obb.vert):
            return False
        return True


    else:
        # Should still make intersecting tests
        return intersection(objA, objB) is not None

def intersection(objA, objB):
    T = (type(objA), type(objB))

    if T == (LineSeg, LineSeg):
        return line_segments_intersection(objA, objB)

    elif T == (Line, Line):
        return lines_intersection(objA, objB)

    elif T == (Line, AABB):
        return line_aabb_intersection(objA, objB)

    elif T == (LineSeg, AABB):
        return line_segment_aabb_intersection(objA, objB)

    elif T == (Ray, AABB):
        return ray_aabb_intersection(objA, objB)

    elif T == (Ray, LineSeg):
        return ray_line_segment_intersection(objA, objB)

    elif T == (Line, Circle):
        return line_circle_intersection(objA, objB)

    else:
        print(f"Error: intersection test not defined for {T}")
        sys.exit()


def lines_intersection_barycentric(AB, CD):
    A, B, C, D = AB.a, AB.b, CD.a, CD.b
    epsilon = 0.01

    v1 = B - A
    n1 = Point(-v1.y, v1.x) * (1/norm(v1))
    n1C = dot(C - A, n1)
    n1D = dot(D - A, n1)
    if abs(n1D - n1C) < epsilon:
        return None
    u = (0 - n1C)/(n1D - n1C) # coefficient of CD to the point of intersection

    # Could t be computed from u?
    v2 = D - C
    n2 = Point(-v2.y, v2.x) * (1/norm(v2))
    n2A = dot(A - C, n2)
    n2B = dot(B - C, n2)
    if abs(n2B - n2A) < epsilon:
        return None
    t = (0 - n2A)/(n2B - n2A) # coefficient of AB to the point of intersection

    return (t, u)
    

def lines_intersection(AB, CD):
    """ Lines are determined by segments AB and CD.
        Returns the point of intersection.
        Returns None if they are parallel to an extent given by epsilon
        (to avoid hugely distant points of intersection).

        BUGS/PROBLEMS:
            close to parallel but intersecting lines which intersect close to their
            segment definitions should be considered intersecting, but currently they
            are not
    """
    A, B, C, D = AB.a, AB.b, CD.a, CD.b
    epsilon = 0.01

    v1 = B - A
    n = Point(-v1.y, v1.x) * (1/norm(v1))
    
    nC = dot(C - A, n)
    nD = dot(D - A, n)

    if abs(nD - nC) < epsilon:
        return None

    return C + ((0 - nC)/(nD - nC))*(D - C)


def line_segments_intersection(AB, CD):
    """ Computes the lines intersection in barycentric coordinates w/r/t each point pair,
        and then does a range check to restrict to segments.
    """
    intersection = lines_intersection_barycentric(AB, CD)
    if intersection is None:
        return None
    t, u = intersection
    if 0 <= t and t <= 1 and 0 <= u and u <= 1:
        return AB.a + t*(AB.b - AB.a)

def ray_line_segment_intersection(ray, seg):
    intersection = lines_intersection_barycentric(ray, seg)
    if intersection is None:
        return None
    t, u = intersection
    if 0 <= t and 0 <= u and u <= 1:
        return ray.a + t*(ray.b - ray.a)




def line_aabb_intersection(AB, box):
    intersections = [lines_intersection_barycentric(AB, seg) for seg in box.segments()]
    intersections = [s for s in intersections if s and 0 <= s[1] <= 1]
    if len(intersections) == 0:
        return None
    return [AB.a + t*(AB.b - AB.a) for t,u in intersections]


def ray_aabb_intersection(AB, box):
    """ With 2-barycentric coordinates, this is a very simple change.
        If a ray intersects any polygon it intersects a side of that polygon.
        Edge cases:
            what about the ray coinciding with a box segment? This will then depend on how line intersections are done as well...
        Gives line segment intersection data
        BUGS/PROBLEMS:
            AABB-line/ray/segment test is not robust.
            Double-intersection -> origin is in box
                    does not actually hold ...
    """
    intersections = [lines_intersection_barycentric(AB, seg) for seg in box.segments()]
    intersections = [s for s in intersections if s and s[0] >= 0 and (0 <= s[1] <= 1)]
    if len(intersections) == 0:
        return None
    elif len(intersections) == 1:
        return [AB.a, AB.a + intersections[0][0]*(AB.b - AB.a)]
    else:
        return [AB.a + t*(AB.b - AB.a) for t,u in intersections]


def line_segment_aabb_intersection(AB, box):
    """ Line segments may be inside the box (still intersecting)
        yet not intersecting any boundary segment.
        So neither [Segment intersects any boundary segment]
            nor    [Either point end of segment is on box]
        are sufficient.
    """
    intersections = [lines_intersection_barycentric(AB, seg) for seg in box.segments()]
    intersections = [s for s in intersections if s and (0 <= s[0] <= 1) and (0 <= s[1] <= 1)]
    if len(intersections) == 0:
        if intersecting(AB.a, box) or intersecting(AB.b, box):
            return [AB.a, AB.b]
        return None
    return [AB.a + t*(AB.b - AB.a) for t,u in intersections]


def point_in_polygon(point, poly, plotting=False):
    """
        Point ray parity
        If a ray which is not parallel to any edge of the polygon is sent from the point,
        then if the parity of the number of intersections the ray makes with a polygon segment is odd,
        the point is in the polygon, otherwise not in.
        BUGS/PROBLEMS:
            Degeneracies when the ray is parallel to an edge, and it goes through the edge
            Point can be on the polygon boundary
        Can do:
            Convex partition (could cache this) then simple convex inclusion
    """
    # ray = Ray(point, point + Point(0, 1))
    ray = Ray(point, point + Point.random(1) -  Point(0.5, 0.5))
    while ray.a == ray.b:
        ray = Ray(point, point + Point.random(1) -  Point(0.5, 0.5))

    if plotting:
        plot(ray, color='k')

    count = 0
    for seg in poly.segments():
        intersect = intersection(ray, seg)
        if intersect:
            if plotting:
                plot(intersect, color='k')
            count += 1
    return count % 2 == 1
    

def convex_polygon_intersection(polyA, polyB):
    
    """ If intersecting, returns the minimal separating vector for polyA """

    min_overlap = float('inf')
    min_separating_axis = None
    separate_poly = polyA

    for axis_poly, other_poly in [(polyA, polyB), (polyB, polyA)]:
        for seg in axis_poly.segments():
            axis = normal_perp(seg.b - seg.a)

            min_axis_poly = 0
            max_axis_poly = max(dot(p - seg.a, axis) for p in axis_poly.points)
            min_other_poly = min(dot(p - seg.a, axis) for p in other_poly.points)
            max_other_poly = max(dot(p - seg.a, axis) for p in other_poly.points)

            if max_other_poly < min_axis_poly or min_other_poly > max_axis_poly:
                return None

            min_sep = min([min_axis_poly - max_other_poly, min_other_poly - max_axis_poly], key=lambda x:abs(x))
            if abs(min_sep) < abs(min_overlap):
                min_overlap = min_sep
                min_separating_axis = axis
                separate_poly = other_poly

    if separate_poly is polyA:
        return min_overlap * min_separating_axis
    else:
        return -min_overlap * min_separating_axis
    

def line_circle_intersection(line, circle):
    """ Currently, not intersecting if tangent to the circle. """
    n = normal_perp(line.b - line.a)
    nline = normalized(line.b - line.a)
    center_dist = dot(circle.point - line.a, n)
    if abs(center_dist) >= circle.radius:
        return None

    rise = sqrt(circle.radius ** 2 - center_dist ** 2)
    return [circle.point - center_dist * n + rise * nline, circle.point - center_dist * n - rise * nline]
    


