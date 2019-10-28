#
# Triangulation and partitioning
# Convex hulls
# Bounding boxes
#

from matplotlib import pyplot as plt
from shapes import *
from operations import *
from plotting import *
from intersections import *
import subprocess
import sys
from collections import namedtuple
from itertools import product

# TODO ###########################################
#
# Partitions and graphs
# Partition is of a polygon, parts are polygons formed from indices into
# the list representation of the parent polygon.
# Technically neighbours can be calculated, but store this information to form a partition
# graph which can be navigated e.g. in the method for convex partition by triangulation
# and elimination of unneccessary edges.
#   - Find
#
#
# What about point-introducing partitions, or partitions which allow line-segment points?
#
##################################################


from collections import defaultdict

class PartitionGraph:
    def __init__(self, poly, subpolys):
        self.points = poly.points
        self.parts = subpolys
        self.parts_adjacency_list = self.calculate_edges()
        # An adjacency matrix would be huge and sparse

    def calculate_edges(self):

        # Edges in a partition graph consist of the index of the other part, and the index (w/r/t the list of indices into the polygon forming the part) of the edge that they are coincident.
        #
        # The adjacency list is in the form:
        #   <part index> : [(<other part>, <coincident edge index of part>), ...]
        #   ...


        parts_adjacency_list = [[] for part in self.parts]

        # Use a dictionary to store a list of the coincident parts and the index of their coincident edge, for each
        # edge which coincides with a part.
        edge_dict = defaultdict(list)
        for part_number, part in enumerate(self.parts):
            for i in range(len(part)):
                edge_index_a = part[i]
                edge_index_b = part[(i + 1) % len(part)]
                # print(f"checking edge {edge_index_a} {edge_index_b}")
                edge_dict[tuple(sorted([edge_index_a, edge_index_b]))].append((part_number, i))
                

        for _, coincident_list in edge_dict.items():
            for part_one, part_one_coincident_edge_index in coincident_list:
                for part_two, __ in coincident_list:
                    if part_one != part_two:
                        parts_adjacency_list[part_one].append((part_two, part_one_coincident_edge_index))

        return parts_adjacency_list

    def part_polys(self):
        """ Returns a list of polygons for each part in the same order """
        return [Poly([self.points[index] for index in part]) for part in self.parts]


def plot_partition_graph(partition_graph, **kwargs):
    polys = partition_graph.part_polys()

    adj_list = partition_graph.parts_adjacency_list

    for i, adjacencies in enumerate(adj_list):
        for connection in adjacencies:
            centroid1 = barycentric_to_cartesian(polys[i], [1 for _ in polys[i]])
            centroid2 = barycentric_to_cartesian(polys[connection[0]], [1 for _ in polys[connection[0]]])

            plot([centroid1, centroid2], **kwargs)
            plot(LineSeg(centroid1, centroid2), **kwargs)

            # plot(intersection(Line(partition_graph.points[partition_graph.parts[i][connection[1]]],
            #         partition_graph.points[partition_graph.parts[i][(connection[1] + 1) % len(partition_graph.parts[i])]]),
            #         Line(centroid1, centroid2)),  s=25, color='lime')


def convex_partition(poly):
    """
        Triangulate then absord triangles/convex parts together until this can no longer be done.
        Take the triangulation graph
    """
    tri_indices = triangulation_indices(poly)
    partition_graph = PartitionGraph(poly, tri_indices)
    adj_list = partition_graph.parts_adjacency_list

    # do processing of the partitions and partition graph
    # partition graph is in an adjacency list format, so it is harder

    # In a convex partition, each edge can maximally be a connection between two parts.
    # Keep finding edges between two parts and removing them if:
    #   - the polygon is convex at the points the edge connects which have only this edge connecting


#     part_index_map = {}
#     for i in range(len(adj_list)):
#         part_index_map[i] = i
#     # To alter the partition graph, the when two parts are joined, this function maps the highest index to the lowest index
#     def join_parts(part_one, part_two):
#         if part_two > part_one:
#             part_index_map[part_two] = part_index_map[part_one]
#         else:
#             part_index_map[part_one] = part_index_map[part_two]

    
#     while True:
#         for part_index, coincident_list in enumerate(adj_list):
#             for other_part_index, coincident_edge_index in coincident_list:
#                 t1 = Triangle(

    return partition_graph


def triangulation_indices(poly):
    if len(poly) < 3:
        print("can only get triangulation graph for polygon with n >= 3")
        sys.exit()

    cut_indices = list(range(len(poly)))
    triangle_indices = []
    while len(cut_indices) > 3:
        for i in range(len(cut_indices)):
            tri_index_center = cut_indices[i]
            tri_index_left = cut_indices[(i - 1) % len(cut_indices)]
            tri_index_right = cut_indices[(i + 1) % len(cut_indices)]

            p_center = poly[tri_index_center]
            p_left = poly[tri_index_left]
            p_right = poly[tri_index_right]

            # plot(poly, color='k')
            # plot([poly[index] for index in cut_indices], color='y')
            # plot(Ray(p_center, p_left), color='b')
            # plot(Ray(p_center, p_right), color='r')
            # plt.show()

            if (det(p_left - p_center, p_right - p_center) > 0
                    and all(not intersecting(LineSeg(p_left, p_right), seg) for seg in set(poly.lines()) - {LineSeg(p_left, p_center), LineSeg(p_center, p_right)})):
                # >>>> buggy, remove at index
                cut_indices.remove(cut_indices[i])
                triangle_indices.append([tri_index_left, tri_index_center, tri_index_right])
                break
    triangle_indices.append(cut_indices)

    return triangle_indices



def triangulate(poly):
    if len(poly) < 3:
        print("can only triangulate polygon with n >= 3")
        sys.exit()
    cut_poly = Poly([p for p in poly])
    triangles = []
    while len(cut_poly) > 3:
        for i in range(len(cut_poly)):
            p1 = cut_poly[(i-1) % len(cut_poly)]
            p2 = cut_poly[(i) % len(cut_poly)]
            p3 = cut_poly[(i+1) % len(cut_poly)]
            # plot([Ray(seg.a, seg.b) for seg in poly.segments()], color='k')
            # plot(cut_poly, color='r')
            # plot(Triangle(p1, p2, p3), color='k')
            # plot(p2, color='b')
            # plot(p1, color='k')
            # plt.show()
            if (det(p1 - p2, p3 - p2) > 0
                    and all(not intersecting(LineSeg(p1, p3), seg) for seg in set(poly.lines()) - {LineSeg(p1, p2), LineSeg(p2, p3)})):
                cut_poly.points.remove(p2)
                triangles.append(Triangle(p1, p2, p3))
                break
    triangles.append(Triangle(*cut_poly))
    return triangles

def triangulate_animate(poly, animate_file=""):
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
                    plot(poly, color='k')
                    plot(triangles, color='r')
                    plot(Triangle(p1, p2, p3), color='g')
                    plt.savefig(f"/tmp/{animate_file}_{count}.png")
                    plt.show()
                    count += 1
                break
            elif animate_file != "":
                plot(poly, color='k')
                plot(triangles, color='r')
                plot(Triangle(p1, p2, p3), color='y')
                plt.savefig(f"/tmp/{animate_file}_{count}.png")
                plt.show()
                count += 1
    triangles.append(Triangle(*cut_poly))
    if animate_file != "":
        plot(poly, color='k')
        plot(triangles, color='r')
        plot(Triangle(*cut_poly), color='g')
        plt.savefig(f"/tmp/{animate_file}_{count}.png")
        plt.show()

        plot(triangles, color='r')
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


def minimal_bounding_circle(points):
    """ Is incorrect. Something like this should be proven.
    """
    square_diameter, point = max([(dot(p - q, p - q), midpoint(p, q)) for p,q in product(points, repeat=2)],
                                  key=lambda x:x[0])
    return Circle(point, sqrt(square_diameter)/2)
