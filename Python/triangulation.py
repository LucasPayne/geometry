"""
    Triangulation and partitioning
    Convex hulls
    Bounding boxes
"""

from matplotlib import pyplot as plt
from shapes import *
from operations import *
from plotting import *
from intersections import *
import subprocess
import sys
from collections import namedtuple
from itertools import product
from math import acos, exp, sqrt

# TODO ###########################################
#
# Partitions and graphs
# Partition is of a polygon, parts are polygons formed from indices into
# the list representation of the parent polygon.
# Technically neighbours can be calculated, but store this information to form a partition
# graph (dual graph) which can be navigated e.g. in the method for convex partition by triangulation
# and elimination of unneccessary edges.
#   - Find
#
#
# What about point-introducing partitions, or partitions which allow line-segment points?
#               ^ These are called Steiner vertices
##################################################


from collections import defaultdict

class PartitionGraph:
# {{{
    """ 
        Class for the dual graph of a partition of a polygon.

        parts_adjacency_list
        The adjacency list is in the form:
          <part index> : [(<other part>, <coincident edge index of part>), ...]
          ...

        edge_coincidence_dict
        This also stores a dictionary
            (low edge index, high edge index) : [(part number, index into part where this edge is), ...]
            ...
    """

    def dual_points(self):
        """ Euclidean locations of the dual points (centroids of the parts) """
        part_polys = self.part_polys()
        return [barycentric_to_cartesian(part_polys[i], [1 for _ in part_polys[i]]) for i in range(len(part_polys))]

    def __init__(self, poly, subpolys):
        self.points = poly.points
        self.parts = subpolys
        self.calculate_edges()
        # An adjacency matrix would be huge and sparse
        # So what about a dictionary?

    def calculate_edges(self):

        # Edges in a partition graph consist of the index of the other part, and the index (w/r/t the list of indices into the polygon forming the part) of the edge that they are coincident.

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

        self.edge_coincidence_dict = edge_dict

        for _, coincident_list in edge_dict.items():
            for part_one, part_one_coincident_edge_index in coincident_list:
                for part_two, __ in coincident_list:
                    if part_one != part_two:
                        parts_adjacency_list[part_one].append((part_two, part_one_coincident_edge_index))

        self.parts_adjacency_list = parts_adjacency_list

    def part_polys(self):
        """ Returns a list of polygons for each part in the same order """
        return [Poly([self.points[index] for index in part]) for part in self.parts]
# }}}

def plot_partition_graph(partition_graph, **kwargs):
# {{{
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
# }}}

# def convex_partition_attempt_2(poly):
# # {{{
#     # ...

#     # >>> Line segment intersect at endpoints

#     if len(poly) == 3:
#         return [poly]

#     for i in range(len(poly)):
#         pleft = poly[(i - 1) % len(poly)]
#         pcenter = poly[i]
#         pright = poly[(i + 1) % len(poly)]
        

#         if det(pright - pcenter, pleft - pcenter) > 0:
#             last_pother_index = None
#             collecting = False
#             for j in range(len(poly)):
#                 if j != (i - 1) % len(poly) and j != i and j != (i + 1) % len(poly):
#                     pother = poly[j]
#                     if all(not intersecting(LineSeg(pcenter, pother), seg) for seg in poly.segments()):
#                         collecting = True
#                         last_pother_index = j
#                     elif collecting:
#                         break

#             piece_one_points = []
#             k = last_pother_index
#             while k != i:
#                 piece_one_points.append(poly[k])
#                 k = (k + 1) % len(poly)
#             piece_one_points.append(poly[i])
#             piece_two_points = []
#             k = last_pother_index
#             while k != i:
#                 piece_two_points.append(poly[k])
#                 k = (k - 1) % len(poly)
#             piece_two_points.append(poly[i])

#             piece_one = Poly(piece_one_points)
#             piece_two = Poly(piece_two_points)

#             plot(poly, color='k')
#             plot(Ray(pcenter, poly[last_pother_index]), color='r')
#             plot(piece_one, color='b')
#             plt.show()

#             return convex_partition(piece_one) + convex_partition(piece_two)

#     return [poly] # must be convex
# # }}}

# def convex_partition_attempt_1(poly):
# {{{
#     """ From triangulation. Should focus on efficient triangulation first so there is a point to this.
#     """
#     tri_indices = triangulation_indices(poly)
#     partition_graph = PartitionGraph(poly, tri_indices)
#     adj_list = partition_graph.parts_adjacency_list
#     coincidence_dict = partition_graph.edge_coincidence_dict
#     tris = partition_graph.parts

#     Adict = {}
#     Bdict = {}
#     Cdict = {}
#     Ddict = {}
#     for edge in coincidence_dict.keys():
#         coincident_parts, coincident_part_edge_indices = zip(*coincidence_dict[edge])
#             # it should be
#             part_one = tris[coincident_parts[0]]
#             part_two = tris[coincident_parts[1]]
#             i = coincident_part_edge_indices[0]
#             j = coincident_part_edge_indices[1]

#             Adict[edge] = poly[part_one[(i + 2) % 3]] - poly[edge[0]]
#             Bdict[edge] = poly[part_one[(i + 2) % 3]] - poly[edge[1]]
#             Cdict[edge] = poly[part_two[(j + 2) % 3]] - poly[edge[0]]
#             Ddict[edge] = poly[part_two[(j + 2) % 3]] - poly[edge[1]]


#     print(Adict)

#     for edge in coincidence_dict.keys():
#         coincident_parts, coincident_part_edge_indices = zip(*coincidence_dict[edge])
#         plot(poly, color='k')
#         plot(Ray(poly[edge[0]], poly[edge[1]]), color='b')
#         print(Adict[edge])
#         print(Bdict[edge])

#         # plot(Ray(poly[edge[0]], poly[edge[0]] + Adict[edge]), color='r')
#         # plot(Ray(poly[edge[1]], poly[edge[1]] + Bdict[edge]), color='r')

#         plt.show()
#     return partition_graph
# }}}

def triangulation(poly):
# {{{
    """ Returns the indices in the poly of the points of the triangles in anticlockwise order,
        [(a1, a2, a3), (b1, b2, b3), ...]
    """
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
# }}}

def triangulation_dual_graph(poly, tri_indices):
# {{{
    return PartitionGraph(poly, tri_indices)
# }}}

def three_color_triangulation_dual_graph(dual_graph):
# {{{
    """ Returns a list of 0, 1, 2 values ("colors"),
        one for each node in the dual graph (triangle in the triangulation).
    """
    colors = ['_' for i in range(len(dual_graph.parts))]
    cur_node = 0
    while len(dual_graph.parts_adjacency_list[cur_node]) > 2:
        cur_node += 1
    colors[cur_node] = 0
    recur_three_color_triangulation_dual_graph(dual_graph, colors, cur_node)
    return colors
def recur_three_color_triangulation_dual_graph(dual_graph, colors, cur_node):
    """ Returns a list of 'r', 'g', 'b' values,
        one for each node in the dual graph.
    """
    count = 0
    for next_node, _ in dual_graph.parts_adjacency_list[cur_node]:
        if colors[next_node] == '_':
            count += 1
            colors[next_node] = (colors[cur_node] + count) % 3
            recur_three_color_triangulation_dual_graph(dual_graph, colors, next_node)
# }}}

def triangulation_triangles(poly):
# {{{
    # Gives the parts of the triangulation as Poly objects.
    if len(poly) < 3:
        print("can only triangulate polygon with n >= 3")
        sys.exit()

    tri_indices = triangulation(poly)
    return [Poly([poly[index] for index in tri]) for tri in tri_indices]
# }}}

def triangulation_animate(poly, animate_file=""):
# {{{
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
            if (det(p1 - p2, p3 - p2) > 0
                    and all(not intersecting(LineSeg(p1, p3), seg) for seg in set(poly.lines()) - {LineSeg(p1, p2), LineSeg(p2, p3)})):
                cut_poly.points.remove(p2)
                triangles.append(Triangle(p1, p2, p3))
                if animate_file != "":
                    plot(poly, color='k')
                    plot(triangles, color='r')
                    plot(Triangle(p1, p2, p3), color='g')
                    plt.savefig(f"/tmp/{animate_file}_{count}.png")
                    plt.clf()
                    count += 1
                break
            elif animate_file != "":
                plot(poly, color='k')
                plot(triangles, color='r')
                plot(Triangle(p1, p2, p3), color='y')
                plt.savefig(f"/tmp/{animate_file}_{count}.png")
                plt.clf()
                count += 1
    triangles.append(Triangle(*cut_poly))
    if animate_file != "":
        plot(poly, color='k')
        plot(triangles, color='r')
        plot(Triangle(*cut_poly), color='g')
        plt.savefig(f"/tmp/{animate_file}_{count}.png")
        plt.clf()

        plot(triangles, color='r')
        plt.savefig(f"/tmp/{animate_file}_{count}.png")
        plt.clf()

        subprocess.call([*"convert -delay 20 -loop 0".split(" "), f"/tmp/{animate_file}_*.png", f"images/{animate_file}.gif"])
        subprocess.call(["rm", *[f"/tmp/{animate_file}_{i}.png" for i in range(1, count+1)]])
    return triangles
# }}}



# >>>
# Algorithms that work with polygons should work with _traversals_ of polygons.
# Traversals are generators which work with the graph of the polygon.
# In this way, the same algorithms can traverse explicit polygons, and hulls represented
# only as fillings ("masking" the polygon). Maybe this idea could extend to other things, so that
# shape information (partitioning, bounding volumes) can be represented relative to the object, but
# still work as the shapes they form.


def convex_hull_poly(poly):
    """
        Returns the "fillings" of the convex hull, pairs of indices into the polygon
        which, in the same order, cover a concavity,
        and the number of vertices on the convex hull.

        Main use is for the computation of data for the HulledPoly object.

        NOTES:
            What about using a stack? Why not look up the polygon indices and wipe out fillings when the stack drops back over them?
    """
    class radial_point(Point):
        def __lt__(self, other):
            return det(self, other) < 0
    # Get point with minimal y then x component.
    start_point = min(poly, key=lambda p: (p.y, p.x))

    # Radially sort the polygon from this starting point
    sorted_indices = sorted(range(len(poly)), key=lambda i: radial_point(poly[i].x - start_point.x, poly[i].y - start_point.y))

    include_bits = [True for _ in sorted_indices]
    count = len(include_bits)
    include_bits[0] = True
    include_bits[1] = True
    for i in range(2, len(sorted_indices)):
        while True:
            j = i - 1
            while not include_bits[j]:
                j -= 1
            k = j - 1
            while not include_bits[k]:
                k -= 1
            if det(poly[sorted_indices[i]] - poly[sorted_indices[j]], poly[sorted_indices[k]] - poly[sorted_indices[j]]) < 0:
                break
            include_bits[j] = False
            count -= 1

    # Now, doing the inverse permutation on the include bits will
    # allow a pass to go over vertex indices and build up cyclic strings of Falses,
    # giving the fillings.
    # The way this is done here is quite horrible but it appears to work.
    poly_include_bits = list(list(zip(*sorted(zip(sorted_indices, include_bits))))[1])
    i = 0
    for _ in range(len(poly)):
        if poly_include_bits[i] and not poly_include_bits[(i + 1) % len(poly)]:
            got_i = True
            break
        i += 1
    fillings = []
    if got_i:
        filling_start = None
        while poly_include_bits[i] is not None :
            if filling_start is None:
                if not poly_include_bits[i]:
                    filling_start = (i - 1) % len(poly)
            elif poly_include_bits[i]:
                fillings.append((filling_start, i))
                filling_start = None
                poly_include_bits[i] = None
            i = (i + 1) % len(poly)
    return (fillings, count)

        

def animate_convex_hull(points, animate_file=""):
    # {{{
    origin = min(points, key=lambda p:p.y)
    
    # Sorting radially: since it is on the convex hull with minimum y component,
    # sort by the angle the points make from the x axis.
    # Naive: take arc-cosines of the x-component of the points normalized
    #     sort anti-clockwise
    # [it worked but should rather use a determinant check]

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
# }}}

def minimal_obb(convex_hull, plotting=False):
# {{{
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
# }}}

def minimal_bounding_circle(points):
# {{{
    """ Is incorrect. Something like this should be proven.
    """
    square_diameter, point = max([(dot(p - q, p - q), midpoint(p, q)) for p,q in product(points, repeat=2)],
                                  key=lambda x:x[0])
    return Circle(point, sqrt(square_diameter)/2)
# }}}
