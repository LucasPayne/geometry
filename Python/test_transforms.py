
def test_obb_transform():
    """ Orthogonal transformations of OBBs """
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
    """ Generate orthonormal coords """
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
    """ Orthogonal transformations of points """
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
