
def test_obb():
    """ Minimal OBBs of two point clouds, and visualization """
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

def test_obb_points():
    """ Multiple Point-OBB ! """
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
    """ >>> NOT WORKING. """
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

