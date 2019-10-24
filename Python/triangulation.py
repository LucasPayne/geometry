#
# Triangulation and partitioning
# Convex hulls
# Bounding boxes
#

from matplotlib import pyplot as plt
from shapes import *
from operations import *
from plotting import *
import subprocess
import sys


def triangulate(poly, animate_file=""):
    """ Ear-clipping """
    if len(poly) < 3:
        print("can only triangulate polygon with n >= 3")
        sys.exit()
    cut_poly = Poly([p for p in poly])
    triangles = []
    if animate_file != "":
        count = 1
    while len(cut_poly) > 3:
        for i in range(len(cut_poly)):
            p1 = cut_poly[(i-1) % len(cut_poly)]
            p2 = cut_poly[(i) % len(cut_poly)]
            p3 = cut_poly[(i+1) % len(cut_poly)]
            if (all(not intersecting(LineSeg(p1, p3), seg) for seg in set(poly.lines()) - {LineSeg(p1, p2), LineSeg(p2, p3)})
                    and all(not intersecting(p, Triangle(p1, p2, p3)) for p in poly.points)):
                cut_poly.points.remove(p2)
                triangles.append(Triangle(p1, p2, p3))
                if animate_file != "":
                    plot(poly, 'k')
                    plot(triangles, 'r')
                    plot(Triangle(p1, p2, p3), 'g')
                    plt.savefig(f"/tmp/{animate_file}_{count}.png")
                    plt.show()
                    count += 1
                break
            elif animate_file != "":
                plot(poly, 'k')
                plot(triangles, 'r')
                plot(Triangle(p1, p2, p3), 'y')
                plt.savefig(f"/tmp/{animate_file}_{count}.png")
                plt.show()
                count += 1
    triangles.append(Triangle(*cut_poly))
    if animate_file != "":
        plot(poly, 'k')
        plot(triangles, 'r')
        plot(Triangle(*cut_poly), 'g')
        plt.savefig(f"/tmp/{animate_file}_{count}.png")
        plt.show()

        plot(triangles, 'r')
        plt.savefig(f"/tmp/{animate_file}_{count}.png")
        plt.show()

        subprocess.call([*"convert -delay 20 -loop 0".split(" "), f"/tmp/{animate_file}_*.png", f"images/{animate_file}.gif"])
        subprocess.call(["rm", *[f"/tmp/{animate_file}_{i}.png" for i in range(1, count+1)]])
    return triangles


from math import acos
from math import exp
def convex_hull(points, animate_file=""):
    origin = min(points, key=lambda p:p.y)
    
    # Sorting radially: since it is on the convex hull with minimum y component,
    # sort by the angle the points make from the x axis.
    # Naive: take arc-cosines of the x-component of the points normalized
    #     sort anti-clockwise

    radial_points = [origin] + sorted(set(points) - {origin}, key=lambda p: -acos(normalized(p - origin).x))

    if animate_file != "":
        for i in range(len(radial_points)):
            ip = (i + 1) % len(radial_points)
            plot(LineSeg(radial_points[i], radial_points[ip]), color='b')
        plt.savefig(f"/tmp/{animate_file}_0.png")
        plt.show()

    cur_gon = list(range(len(radial_points)))
    count = 1
    while True:
        updated = False
        new_gon = []
        for i in range(len(cur_gon)):
            ip = (i - 1) % len(cur_gon)
            ipp = (i + 1) % len(cur_gon)
            # print(ip, i, ipp)
            # print(cur_gon[ip], cur_gon[i], cur_gon[ipp])
            area = Triangle(radial_points[cur_gon[ip]], radial_points[cur_gon[i]], radial_points[cur_gon[ipp]]).area()
            if area >= 0:
                new_gon.append(cur_gon[i])
            else:
                updated = True
        # print(cur_gon)
        cur_gon = new_gon
        # print(cur_gon)

        if animate_file != "":
            for i in range(len(cur_gon)):
                point1 = radial_points[cur_gon[i]]
                point2 = radial_points[cur_gon[(i + 1)%len(cur_gon)]]
                plot(LineSeg(point1, point2), color='r', alpha=(1/(1 + exp(-count - 1))))
            plt.savefig(f"/tmp/{animate_file}_{count}.png")
            plt.show()
        count += 1
        if not updated:
            break
    if animate_file != "":
        subprocess.call([*"convert -delay 20 -loop 0".split(" "), f"/tmp/{animate_file}_*.png", f"images/{animate_file}.gif"])
        subprocess.call(["rm", *[f"/tmp/{animate_file}_{i}.png" for i in range(1, count+1)]])
    
    # >>> Should get the convex hull in the right orientation in the first place
    return Poly([radial_points[i] for i in cur_gon][::-1])


from math import sqrt
def minimal_obb(convex_hull, plotting=False):
    """
        Brute force:
        p: #points
        n: #points on convex hull
        O(plogp + n^2)
        
        O(plogp + n) is possible with rotating calipers.
            [n <= p so O(plogp)]
    """

    cur_min_area = float('inf')
    cur_depth = None
    cur_horiz_left = None
    cur_horiz_right = None
    cur_seg = None

    points = convex_hull.points

    if plotting:
        plot(convex_hull, color='b')

    for seg in convex_hull.segments():

        right = normalized(seg.b - seg.a)
        up = perp(right)

        depth = max(dot(p - seg.a, up) for p in points)
        horiz_left = min(dot(p - seg.a, right) for p in points)
        horiz_right = max(dot(p - seg.a, right) for p in points)

        area = depth * (horiz_right - horiz_left)
        if area < cur_min_area:
            cur_min_area = area
            cur_depth = depth
            cur_horiz_left = horiz_left
            cur_horiz_right = horiz_right
            cur_seg = seg

        if plotting:
            obb = OBB(seg.a + horiz_left * right,
                      right,
                      horiz_right - horiz_left,
                      depth)
            plot(obb, color='r')

    obb = OBB(cur_seg.a + cur_horiz_left * normalized(cur_seg.b - cur_seg.a),
              normalized(cur_seg.b - cur_seg.a),
              cur_horiz_right - cur_horiz_left,
              cur_depth)
    return obb


from itertools import product
def minimal_bounding_circle(points):
    """ Is incorrect. Something like this should be proven.
    """
    square_diameter, point = max([(dot(p - q, p - q), midpoint(p, q)) for p,q in product(points, repeat=2)],
                                  key=lambda x:x[0])
    return Circle(point, sqrt(square_diameter)/2)
    

