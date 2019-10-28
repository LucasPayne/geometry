from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *
from randomize import *

def test_collision_convex():
    """ Creates a GIF of a collision between moving convex polygons """
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
