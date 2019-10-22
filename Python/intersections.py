#
# Intersection tests
# Intersecting?
# Intersection information: contact manifold, overlap polygon
#   (can include polygon collections from polygon intersections, etc.)
#

from shapes import *
from operations import *


def intersecting(objA, objB):
    if type(objA) is Triangle and type(objB) is Circle:
        triangle = objA
        circle = objB
        mink_sum = minkowski(triangle, circle)
        return any([point_in(circle.point, part) for part in mink_sum])

    elif type(objA) is LineSeg and type(objB) is LineSeg:
        a1 = objA.a
        a2 = objA.b
        b1 = objB.a
        b2 = objB.b
        return (((det(b1 - a1, a2 - a1) < 0) != (det(b2 - a1, a2 - a1) <= 0)) and
                ((det(a1 - b1, b2 - b1) < 0) != (det(a2 - b1, b2 - b1) <= 0)))

    elif type(objA) is Point and type(objB) is AABB:
        point, box = objA, objB
        return ((box.origin.x <= point.x <= box.origin.x + box.horiz) and
                (box.origin.y <= point.y <= box.origin.y + box.vert))

    elif type(objA) is Point and type(objB) is Triangle:
        point, A, B, C = objA, objB.A, objB.B, objB.C
        return (det(B - A, point - A) < 0 and
                det(C - B, point - B) < 0 and
                det(A - C, point - C) < 0)

    elif type(objA) is Point and type(objB) is Circle:
        point, circle = objA, objB
        return square_length(point - circle.point) < circle.radius ** 2


def intersection(objA, objB):
    if type(objA) is LineSeg and type(objB) is LineSeg:
        return line_segments_intersection(objA, objB)

    elif type(objA) is Line and type(objB) is Line:
        return lines_intersection(objA, objB)

    elif type(objA) is Line and type(objB) is AABB:
        return line_aabb_intersection(objA, objB)

    elif type(objA) is LineSegment and type(objB) is AABB:
        return line_segment_aabb_intersection(objA, objB)

    elif type(objA) is Ray and type(objB) is AABB:
        return ray_aabb_intersection(objA, objB)


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
            close to parallel but intersecting lines which interesect close to their
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
