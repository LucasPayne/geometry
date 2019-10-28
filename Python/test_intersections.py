from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *
from randomize import *


# Intersecting ? tests
def test_line_segment_intersecting():
    """ LineSeg-LineSeg ? """
# {{{
    while True:
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        # plot(LineSeg(midpoint(seg1.a, seg1.b), midpoint(seg2.a, seg2.b)), 'g')
        plot([seg1, seg2], color='r' if intersecting(seg1, seg2) else 'b')
        plt.show()
# }}}

def test_aabb_intersecting():
    """ AABB-AABB ? """
# {{{
    while True:
        box1 = AABB(Point.random(1), 0.1 + 0.6 * random(), 0.1 + 0.6 * random())
        box2 = AABB(Point.random(1), 0.1 + 0.6 * random(), 0.1 + 0.6 * random())
        plot([box1, box2], color='r' if intersecting(box1, box2) else 'b')
        plt.show()
# }}}

def test_obb_intersecting():
    """ OBB-OBB ? """
# {{{
    while True:
        box1 = OBB(Point.random(1),
                   Point.random(1) - Point(0.5, 0.5),
                   0.1 + 0.6 * random(),
                   0.1 + 0.6 * random())
        box2 = OBB(Point.random(1),
                   Point.random(1) - Point(0.5, 0.5),
                   0.1 + 0.6 * random(),
                   0.1 + 0.6 * random())
        plot([box1, box2], color='r' if intersecting(box1, box2) else 'b')
        plt.show()
# }}}

def test_point_in_polygon():
    """ Point-Polygon ? by casting a ray, and visualization """
# {{{
    for i in [1,2,3,4]:
        poly = make_poly_from_text(f"data/{i}.poly")
        plot(poly, color='k')
        points = [Point.random(1) for _ in range(20)]
        for p in points:
            if point_in_polygon(p, poly, plotting=True):
                plot(p, color='k', s=100)
                plot(p, color='y')
            else:
                plot(p, color='b')
        plt.show()
# }}}

# Intersection ! tests
def test_line_segment_intersection():
    """ LineSeg-LineSeg ! """
# {{{
    while True:
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        intersection_point = line_segments_intersection(seg1, seg2)
        plot([seg1, seg2], color='r' if intersection_point else 'b')
        if intersection_point:
            plt.scatter([intersection_point.x], [intersection_point.y], color='k')
        plt.show()
# }}}

def test_mass_line_segments():
    """ LineSeg-LineSeg-... ! """
# {{{
    while True:
        segments = []
        intersection_points = []
        for i in range(20):
            seg = LineSeg(Point.random(1), Point.random(1))
            for other_seg in segments:
                intersection_point = line_segments_intersection(seg, other_seg)
                if intersection_point:
                    intersection_points.append(intersection_point)
            segments.append(seg)
        for seg in segments:
            plot(seg, color='b')
        for p in intersection_points:
            plt.scatter([p.x], [p.y], color='k')
        plt.show()
# }}}

def test_lines_intersection():
    """ Line-Line ! """
# {{{
    def test(seg1, seg2):
        intersection_point = lines_intersection(seg1, seg2)
        if intersection_point:
            plot([LineSeg(seg1.a, intersection_point), LineSeg(seg1.b, intersection_point),
                  LineSeg(seg2.a, intersection_point), LineSeg(seg2.b, intersection_point)], color='k')
            plot([seg1, seg2], color='r')
            plot([seg1.a, seg1.b, seg2.a, seg2.b], color='r')
            plt.scatter([intersection_point.x], [intersection_point.y], color='k')
        else:
            plot([seg1, seg2], color='b')
        plt.show()

    # test for parallels
    test(LineSeg(Point(1, 1), Point(2, 2)),
         LineSeg(Point(0, 1), Point(1, 2)))
    while True:
        test(LineSeg(Point.random(1), Point.random(1)),
             LineSeg(Point.random(1), Point.random(1)))

# }}}

def test_line_aabb_intersection(kind="line"):
    """ Line-AABB ! """
# {{{
    # intersection_func = ray_aabb_intersection if kind=="ray" else line_aabb_intersection
    intersection_func = {'line':line_aabb_intersection, 'ray':ray_aabb_intersection, 'segment':line_segment_aabb_intersection}[kind]
    def test(box, seg):
        intersection_data = intersection_func(seg, box)
        if intersection_data:
            for p in [seg.a, seg.b]:
                for s in intersection_data:
                    plot(LineSeg(p, s), color='k')
            plot([box, seg], color='r')
            if kind=="ray":
                plot_arrowhead(seg.b, seg.b - seg.a, color='r')
            plot(intersection_data, color='k')
        else:
            if kind=="ray":
                plot_arrowhead(seg.b, seg.b - seg.a, color='b')
            plot([box, seg], color='b')
        plt.show()

    # directional test
    test(AABB(Point(0, 0), 1, 1),
         LineSeg(Point(2, 2), Point(1.5, 1.5)))
    test(AABB(Point(0, 0), 1, 1),
         LineSeg(Point(1.5, 1.5), Point(2, 2)))
    # Internal-out test
    test(AABB(Point(0, 0), 1, 1),
         LineSeg(Point(0.5, 0.1), Point(0.5, -1)))
    test(AABB(Point(0, 0), 1, 1),
         LineSeg(Point(0.9, 0.9), Point(1.1, 1.1)))
    # Internal test
    test(AABB(Point(0, 0), 1, 1),
         LineSeg(Point(0.04, 0.93), Point(0.98, 0.05)))

    while True:
        box = AABB(Point.random(1), 1.0 * random(), 1.0 * random())
        seg = LineSeg(Point.random(2), Point.random(2))
        test(box, seg)
# }}}

def test_ray_aabb_intersection():
    """ Ray-AABB ! """
# {{{
    test_line_aabb_intersection(kind="ray")
# }}}

def test_line_segment_aabb_intersection():
    """ LineSeg-AABB ! """
# {{{
    test_line_aabb_intersection(kind="segment")
# }}}

def test_convex_polygon_intersection():
    """ ConvexPoly-ConvexPoly !, and minimal separating vector """
# {{{
    while True:
        polyA = random_convex_polygon(0, 1, 0, 1, 10, shift=1.25)
        polyB = random_convex_polygon(0, 1, 0, 1, 10, shift=1.25)
        intersect = convex_polygon_intersection(polyA, polyB)

        plot([polyA, polyB], color='r' if intersect else 'b')
        if intersect:
             centroidA = barycentric_to_cartesian(polyA.points, [1 for _ in polyA.points])
             plot(Ray(centroidA, centroidA + intersect), color='k')
             plot(polyA + intersect, color='g')

        plt.show()
# }}}

def test_line_circle_intersection():
    """ Line-Circle !, with a fan of lines """
# {{{
    while True:
        circle = Circle(Point.random(1), 0.3 * random() + 0.05)
        start_line = Line(Point.random(1), Point.random(1))

        circle_intersecting = False

        for i in range(-6, 7):
            line = Line(start_line.a,
                        start_line.a + rotate_vector(start_line.b - start_line.a,
                                                     -pi/4.0 + i * pi/(2.0*12)))
            intersect = intersection(line, circle)
            plot(line, color='r' if intersect else 'b')
            if intersect:
                circle_intersecting = True
                plot(intersect, color='k')

        plot(circle, color='r' if circle_intersecting else 'b')

        plt.show()
# }}}

def test_line_polygon_intersection():
    """ Line-Poly ! """
# {{{
    while True:
        line = Line(Point.random(1), Point.random(1))
        poly = random_convex_polygon(0, 1, 0, 1, 10, shift=1.25)

        intersect = intersection(line, poly)
        plot([line, poly], color='r' if intersect else 'b')
        if intersect:
            plot(intersect, color='k')

        plt.show()
# }}}

