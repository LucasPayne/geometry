from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *

from matplotlib import pyplot as plt
from random import random
import readline
import functools

def test_centroids():
    while True:
        poly = random_poly(randrange(2, 8))
        plot(poly)
        plot(centroid(poly))

        T = random_shape(Triangle)
        plot(T)
        plot(triangle_centroid(T))
        plt.show()

def test_triangulate():
    # poly = Poly([Point(1, 1),
    #              Point(1, 2),
    #              Point(2, 2),
    #              Point(2, 1),
    #              Point(3, 1),
    #              Point(3, -0.5)])


    for i in [1,2,3]:
        poly = make_poly_from_text(f"data/{i}.poly")
        triangles = triangulate(poly, f"triangulation{i}")
        # plot(poly)
        # plot(triangles, 'g')
        # plot(polygon_barycenter(poly))
        # plt.show()


def test_line_segment_intersecting():
    while True:
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        # plot(LineSeg(midpoint(seg1.a, seg1.b), midpoint(seg2.a, seg2.b)), 'g')
        plot([seg1, seg2], 'r' if intersecting(seg1, seg2) else 'b')
        plt.show()
def test_line_segment_intersection():
    while True:
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        intersection_point = line_segments_intersection(seg1, seg2)
        plot([seg1, seg2], 'r' if intersection_point else 'b')
        if intersection_point:
            plt.scatter([intersection_point.x], [intersection_point.y], color='k')
        plt.show()

def test_mass_line_segments():
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


def test_lines_intersection():
    def test(seg1, seg2):
        intersection_point = lines_intersection(seg1, seg2)
        if intersection_point:
            plot([LineSeg(seg1.a, intersection_point), LineSeg(seg1.b, intersection_point),
                  LineSeg(seg2.a, intersection_point), LineSeg(seg2.b, intersection_point)], color='k')
            plot([seg1, seg2], 'r')
            plot([seg1.a, seg1.b, seg2.a, seg2.b], 'r')
            plt.scatter([intersection_point.x], [intersection_point.y], color='k')
        else:
            plot([seg1, seg2], 'b')
        plt.show()

    # test for parallels
    test(LineSeg(Point(1, 1), Point(2, 2)),
         LineSeg(Point(0, 1), Point(1, 2)))
    while True:
        test(LineSeg(Point.random(1), Point.random(1)),
             LineSeg(Point.random(1), Point.random(1)))


def test_line_aabb_intersection(kind="line"):
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
def test_ray_aabb_intersection():
    test_line_aabb_intersection(kind="ray")
def test_line_segment_aabb_intersection():
    test_line_aabb_intersection(kind="segment")


def test_obb():
    while True:
        points = []
        # for _ in range(500):
        #     p = Point.random(random() * random())
        #     points.append(p)
        cluster_offset = Point.random(5)
        points = [Point.random(1) for _ in range(50)]
        points += [Point.random(2) + cluster_offset for _ in range(50)]

        hull = convex_hull(points)
        plot(points, color='b', s=0.5)
        obb = minimal_obb(hull, plotting=True)
        plot(hull, color='lime')
        plot(obb, color='k', linewidth=5)
        plt.show()


def test_bounding_circle():
    while True:
        cluster_offset = Point.random(5)
        points = [Point.random(1) for _ in range(50)]
        points += [Point.random(2) + cluster_offset for _ in range(50)]
        bounding_circle = minimal_bounding_circle(points)
        plot(points, color='b')
        plot(bounding_circle, color='r')
        plt.show()

def test_convex_hull():
    while True:
        points = [Point.random(1) for _ in range(500)]
        convex_hull(points, animate_file="convex_hull_animation")
        plt.show()

def prefix_function(function, prefunction):
    # from SO: hook python module function
    @functools.wraps(function)
    def run(*args, **kwargs):
        prefunction(*args, **kwargs)
        return function(*args, **kwargs)
    return run

def prefix_plt_show():
    plt.gca().set_aspect('equal', adjustable='box')
plt.show = prefix_function(plt.show, prefix_plt_show)

def complete(text, state):
    for cmd in commands:
        if cmd.startswith(text):
            if not state:
                return cmd
            else:
                state -= 1

commands = [name[len("test_"):] for name in dir() if name.startswith("test_")]
for command in commands:
    print(f"\t{command}")

readline.parse_and_bind("tab: complete")
readline.set_completer(complete)

while True:
    inp = input('Enter section name: ')
    if inp in commands:
        eval(f"test_{inp}()")
        break


