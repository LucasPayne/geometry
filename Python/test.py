from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *
from randomize import *

from matplotlib import pyplot as plt
from random import random
import readline
import functools

from math import pi

from matplotlib.animation import FuncAnimation

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


def test_centroids():
# {{{
    while True:
        poly = random_poly(randrange(2, 8))
        plot(poly)
        plot(centroid(poly))

        T = random_shape(Triangle)
        plot(T)
        plot(triangle_centroid(T))
        plt.show()
# }}}

def test_animate_triangulation():
# {{{
    # poly = Poly([Point(1, 1),
    #              Point(1, 2),
    #              Point(2, 2),
    #              Point(2, 1),
    #              Point(3, 1),
    #              Point(3, -0.5)])

    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        triangles = triangulation_animate(poly, f"triangulation{i}")
        # plot(poly)
        # plot(triangles, 'g')
        # plot(polygon_barycenter(poly))
        # plt.show()

# }}}

def test_triangulation():
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        triangles = triangulation_triangles(poly)
        plot(poly, color='k')
        plot(triangles, color='r')
        plt.show()
# }}}

def test_convex_partition():
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        parts = convex_partition(poly)
        plot(parts, color='r')
        plt.show()
# }}}

# >>> move this
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

def test_graph_triangulation():
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")

        tri_indices = triangulation(poly)

        dual_graph = triangulation_dual_graph(poly, tri_indices)

        plot(dual_graph.part_polys(), color='k')

        colors = three_color_triangulation_dual_graph(dual_graph)
        
        for i, tri in enumerate(dual_graph.part_polys()):
            plot_hatch_convex_poly(tri, color=['r','g','b'][colors[i]])

        # plot_partition_graph(dual_graph, color='k')
        # for i, p in enumerate(dual_graph.dual_points()):
        #     plot(p, color=['r', 'g', 'b'][colors[i]])

        plt.show()

# }}}

def test_line_segment_intersecting():
# {{{
    while True:
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        # plot(LineSeg(midpoint(seg1.a, seg1.b), midpoint(seg2.a, seg2.b)), 'g')
        plot([seg1, seg2], color='r' if intersecting(seg1, seg2) else 'b')
        plt.show()
# }}}

def test_line_segment_intersection():
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
# {{{
    test_line_aabb_intersection(kind="ray")
# }}}

def test_line_segment_aabb_intersection():
# {{{
    test_line_aabb_intersection(kind="segment")
# }}}

def test_obb():
# {{{
    while True:

        boxes = []
        for _ in range(2):
            points = []
            clusters_offset = Point.random(2)
            cluster1_offset = Point.random(5)
            cluster2_offset = Point.random(5)
            cluster1_size = 2.0*random() + 0.3
            cluster2_size = 2.0*random() + 0.3
            points = [Point.random(cluster1_size) + clusters_offset + cluster1_offset for _ in range(50)]
            points += [Point.random(cluster2_size) + clusters_offset + cluster2_offset for _ in range(50)]
            hull = convex_hull(points)
            plot(points, color='b', s=0.5)
            plot(hull, color='lime')
            # obb = minimal_obb(hull, plotting=True)
            obb = minimal_obb(hull, plotting=False)
            # plot(obb, color='k', linewidth=5)
            boxes.append(obb)

        plot(boxes, color='r' if intersecting(*boxes) else 'b')

        plt.show()
# }}}

def test_minor_obb_points():
# {{{
    while True:
        points = [Point.random(1) for _ in range(100)]
        obb = OBB(Point.random(1),
                  Point.random(1) - Point(0.5, 0.5),
                  2*random(),
                  2*random())
        for p in points:
            plot(p, color='r' if intersecting(p, obb) else 'b')
        plot(obb, color='k')
        plt.show()
# }}}

def test_bounding_circle():
# {{{
    while True:
        cluster_offset = Point.random(5)
        points = [Point.random(1) for _ in range(50)]
        points += [Point.random(2) + cluster_offset for _ in range(50)]
        bounding_circle = minimal_bounding_circle(points)
        plot(points, color='b')
        plot(bounding_circle, color='r')
        plt.show()
# }}}

def test_convex_hull():
# {{{
    while True:
        points = [Point.random(1) for _ in range(500)]
        convex_hull(points, animate_file="convex_hull_animation")
        plt.show()
# }}}

def test_aabb_intersecting():
# {{{
    while True:
        box1 = AABB(Point.random(1), 0.1 + 0.6 * random(), 0.1 + 0.6 * random())
        box2 = AABB(Point.random(1), 0.1 + 0.6 * random(), 0.1 + 0.6 * random())
        plot([box1, box2], color='r' if intersecting(box1, box2) else 'b')
        plt.show()
# }}}

def test_obb_intersecting():
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

def test_obb_transform():
# {{{
    while True:
        box = OBB(Point.random(1),
                  Point.random(1) - Point(0.5, 0.5),
                  0.1 + 0.6 * random(),
                  0.1 + 0.6 * random())
        frame = CartesianFrame(Point.random(1),
                               Point.random(1) - Point(0.5, 0.5),
                               Point.random(1) - Point(0.5, 0.5))
        frame.orthogonalize_e1()
        frame.normalize()

        # print(f"box: {box}")
        transformed_box = transformed(box, frame)
        # print(f"transformed_box: {transformed_box}")

        
        plot(frame, color='k')
        plot(CartesianFrame.ambient(), color='r')
        plot(box, color='k')
        plot(transformed_box, color='r')

        plt.show()
# }}}

def test_gram_schmidt():
# {{{
    while True:
        coords = CartesianFrame(Point.random(1), 
                                Point.random(1) - Point(0.5, 0.5),
                                Point.random(1) - Point(0.5, 0.5))
        
        plot(coords, color='k')
        coords.orthogonalize_e2()
        plot(coords, color='r')
        plt.show()
# }}}

def test_descartes():
# {{{
    while True:
        coords = CartesianFrame(Point.random(2) - Point(1, 1), 
                                Point.random(3) - Point(1.5, 1.5),
                                Point.random(3) - Point(1.5, 1.5))
        coords.orthogonalize_e2()
        plot(coords, color='k')

        points = [Point.random(3) - Point(1.5, 1.5) for _ in range(3)]
        plot(points, color='k', s=10)

        # >>> show two plots side by side

        axis_coords = CartesianFrame(Point(0, 0), Point(norm(coords.e1), 0), Point(0, norm(coords.e2)))
        plot(axis_coords, color='r')

        transformed_points = []
        for p in points:
            new_p = Point(dot(p - coords.origin, normalized(coords.e1)),
                          dot(p - coords.origin, normalized(coords.e2)))
            transformed_points.append(new_p)

        plot(transformed_points, color='r', s=10)

        plt.show()
# }}}

def test_point_in_polygon():
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

def test_convex_polygon_intersection():
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

def test_collision_convex():
# {{{
    def set_axis():
        plt.axis([0, 2.4, 0, 2.4])
    set_axis()

    polyA = None
    polyB = None
    while True:
        polyA = random_convex_polygon(0, 0.2, 0, 0.2, 10, shift=1.25)
        polyB = random_convex_polygon(0, 1, 0, 1, 30, shift=1.25)
        if not convex_polygon_intersection(polyA, polyB):
            break

    centroidA = barycentric_to_cartesian(polyA.points, [1 for _ in polyA.points])
    centroidB = barycentric_to_cartesian(polyB.points, [1 for _ in polyB.points])
    motion_vector = normalized(centroidB - centroidA) * 0.12

    for count in range(1, 30):
        plot([polyA, polyB], color='k')

        plt.savefig(f"/tmp/animate_collision_convex_{count}.png")
        plt.clf()
        set_axis()

        centroidA = barycentric_to_cartesian(polyA.points, [1 for _ in polyA.points])
        centroidB = barycentric_to_cartesian(polyB.points, [1 for _ in polyB.points])
        polyA = polyA + motion_vector
        intersect = convex_polygon_intersection(polyA, polyB)
        if intersect:
            plot(polyA, color='r')
            plot(Ray(centroidA, centroidA + intersect), color='g')
            polyA = polyA + intersect
    subprocess.call([*"convert -alpha remove -layers OptimizePlus -delay 20 -loop 0".split(" "), f"/tmp/animate_collision_convex_*.png", f"images/collisionconvex.gif"])
    subprocess.call(["mpv", "--loop", "images/collisionconvex.gif"])
# }}}

def test_line_circle_intersection():
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
    while True:
        line = Line(Point.random(1), Point.random(1))
        poly = random_convex_polygon(0, 1, 0, 1, 10, shift=1.25)

        intersect = intersection(line, poly)
        plot([line, poly], color='r' if intersect else 'b')

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


if len(sys.argv) == 2:
    eval(f"test_{sys.argv[1]}()")
    sys.exit()


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
    inp = input('Enter test name: ')
    if inp in commands:
        eval(f"test_{inp}()")
        break


