
from matplotlib import pyplot as plt
from shapes import LineSeg, Point, AABB
from plotting import plot
from abc import ABCMeta, abstractmethod 

import subprocess

from utils import *

def convex_hull_polygon(poly):

    class radial_point(Point):
        def __lt__(self, other):
            return det(self, other) < 0

    # Get point with minimal y then x component.
    start_point_index, start_point = min(enumerate(poly.points), key=lambda ip: (ip[1].y, ip[1].x))

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
    return (fillings, start_point_index, count)


class Graph(metaclass=ABCMeta):
    @abstractmethod
    def coincident_nodes(self, node):
        raise NotImplementedError
    @abstractmethod
    def default_start_node(self):
        raise NotImplementedError

class PolygonGraph(Graph):
    def __init__(self, poly):
        self.poly = poly
    def coincident_nodes(self, index):
        return [(index - 1) % self.poly.n, (index + 1) % self.poly.n]
    def default_start_node(self):
        return 0

class Polygon:
    """ Explicit polygon """

    def __init__(self, points):
        self.points = points
        self.n = len(points)
        self.graph = PolygonGraph(self)

    def __getitem__(self, key):
        """ IMPORTANT:
                Random access to a polygon subclass is by default access to the ordered points of the parent polygon.
        """
        return self.points[key]

    def __len__(self):
        return len(self.points)

    def __iter__(self, start_index=0):
        yield start_index, self.points[start_index]
        index = start_index
        while True:
            index = (index + 1) % self.n
            if index == start_index:
                raise StopIteration
            yield index, self.points[index]


class PolygonHullGraph(Graph):
    def __init__(self, polygon_hull):
        self.polygon_hull = polygon_hull
    def coincident_nodes(self, index):
        skipped_forward = False
        skipped_backward = False
        nodes = []
        for filling in self.polygon_hull.fillings:
            if filling[0] == index:
                nodes.append(filling[1])
                skipped_forward = True
            if filling[1] == index:
                nodes.append(filling[0])
                skipped_backward = True
        if not skipped_forward:
            nodes.append((index + 1) % self.polygon_hull.n)
        if not skipped_backward:
            nodes.append((index - 1) % self.polygon_hull.n)
        return nodes
    def default_start_node(self):
        return self.polygon_hull.hull_point_index

class PolygonHull(Polygon):
    def __init__(self, parent_polygon):
        if type(parent_polygon) is not Polygon:
            raise TypeError("PolygonHull can only be formed from a Polygon")
        self.parent_polygon = parent_polygon
        self.points = parent_polygon.points
        self.n = len(parent_polygon.points)
        self.fillings, self.hull_point_index, self.num_hull_points = convex_hull_polygon(parent_polygon)
        self.graph = PolygonHullGraph(self)
        
    def __iter__(self, start_index=None):
        """ 
            Iteration-based algorithms will work on this subclass, which implements the iterator
            by skipping over the fillings of the convex hull.
            Not defined (may not halt) if the start index is not on the hull
        """
        if start_index is None:
            start_index = self.hull_point_index
        yield start_index, self.points[start_index]
        index = start_index
        while True:
            skipped = False
            for filling in self.fillings:
                if filling[0] == index:
                    index = filling[1]
                    skipped = True
                    break
            if not skipped:
                index = (index + 1) % self.n
            if index == start_index:
                raise StopIteration
            yield index, self.points[index]

def plot_polygon(poly, **kwargs):
    # Never turn around in the traversal, should probably implement uni-directional and bi-directional graphs instead of doing this.
    def get_next_node(cur_node, last_node):
        nodes = poly.graph.coincident_nodes(cur_node)
        for node in nodes:
            if node != last_node:
                return node
    start_node = poly.graph.default_start_node()
    cur_node = start_node
    next_node = get_next_node(cur_node, None)
    while True:
        plot(LineSeg(poly.points[cur_node], poly.points[next_node]), **kwargs)
        if next_node == start_node:
            break
        last_node = cur_node
        cur_node = next_node
        next_node = get_next_node(next_node, last_node)

from operations import dot
def hillclimb_polygon(poly, direction, start_node=None):
    if start_node is None:
        start_node = poly.graph.default_start_node()
    origin = poly.points[start_node]

    def goodness(node):
        return dot(poly.points[node] - origin, direction) 

    current_node = start_node
    while True:
        next_nodes = poly.graph.coincident_nodes(current_node)
        current_good = goodness(current_node)
        optimal_next_node, optimal_next_node_goodness = max(zip(next_nodes, [goodness(node) for node in next_nodes]), key=lambda o: o[1])

        if optimal_next_node_goodness <= current_good:
            return current_node

        current_node = optimal_next_node


lock_axes()

from operations import rotate_vector
import math
for poly_num, poly in enumerate(data_polys()):
    poly = Polygon(poly.points)

    hull = PolygonHull(poly)
    min_x_index = hillclimb_polygon(hull, Point(-1, 0))
    max_x_index = hillclimb_polygon(hull, Point(1, 0))
    min_y_index = hillclimb_polygon(hull, Point(0, -1))
    max_y_index = hillclimb_polygon(hull, Point(0, 1))

    theta = 0.1
    count = 0
    for _ in range(math.ceil((2 * math.pi) / theta)):
        count += 1

        hull = PolygonHull(poly)

        plot_polygon(poly, color='k')
        plot_polygon(hull, color='r') # why recompute the hull? > fix this

        min_x_index = hillclimb_polygon(hull, Point(-1, 0), start_node=min_x_index)
        max_x_index = hillclimb_polygon(hull, Point(1, 0), start_node=max_x_index)
        min_y_index = hillclimb_polygon(hull, Point(0, -1), start_node=min_y_index)
        max_y_index = hillclimb_polygon(hull, Point(0, 1), start_node=max_y_index)

        plot(poly.points[min_x_index], color='b')
        plot(poly.points[max_x_index], color='b')
        plot(poly.points[min_y_index], color='b')
        plot(poly.points[max_y_index], color='b')
   
        aabb = AABB(Point(poly[min_x_index].x, poly[min_y_index].y),
                       poly[max_x_index].x - poly[min_x_index].x,
                       poly[max_y_index].y - poly[min_y_index].y)

        plot(aabb, color='b')
        
        print(f"writing image {count}")
        plt.savefig(f"/tmp/animation_{count}.png")
        plt.clf()
        poly = Polygon([rotate_vector(p - Point(0, 0), 0.1) for p in poly.points])
    
    print(f"making gif {poly_num}")
    subprocess.call([*"convert -delay 20 -loop 0".split(" "), f"/tmp/animation_*.png", f"images/dynamic_aabb_{poly_num}.gif"])
    print("cleaning up")
    subprocess.call(["rm", *[f"/tmp/animation_{i}.png" for i in range(1, count+1)]])
    

# for poly in data_polys():
#     poly = Polygon(poly.points) # do this for the meantime while this is not the official polygon class
#     hull = PolygonHull(poly)

#     plot_polygon(poly, color='k')
#     plot_polygon(hull, color='r')

#     min_x_index = hillclimb_polygon(hull, Point(-1, 0))
#     max_x_index = hillclimb_polygon(hull, Point(1, 0))
#     min_y_index = hillclimb_polygon(hull, Point(0, -1))
#     max_y_index = hillclimb_polygon(hull, Point(0, 1))

#     plot(poly.points[min_x_index], color='b')
#     plot(poly.points[max_x_index], color='b')
#     plot(poly.points[min_y_index], color='b')
#     plot(poly.points[max_y_index], color='b')

    
#     aabb = AABB(Point(poly[min_x_index].x, poly[min_y_index].y),
#                       poly[max_x_index].x - poly[min_x_index].x,
#                       poly[max_y_index].y - poly[min_y_index].y)

#     plot(aabb, color='lime')

#     plt.show()
