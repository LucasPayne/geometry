
def test_animate_triangulation():
    """ Creates GIFs of triangulations for data/ polygons """
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        triangles = triangulation_animate(poly, f"triangulation{i}")
# }}}

def test_animate_convex_hull():
    """ Creates a GIF of a convex hull progressively expanding """
# {{{
    while True:
        points = [Point.random(1) for _ in range(500)]
        convex_hull(points, animate_file="convex_hull_animation")
        plt.show()
# }}}

def test_triangulation():
    """ Displays triangulations of data/ polygons """
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        triangles = triangulation_triangles(poly)
        plot(poly, color='k')
        plot(triangles, color='r')
        plt.show()
# }}}

def test_convex_partition():
    """ Attempts... a convex partition of data/ polygons """
# {{{
    for i in [1,2,4,5]:
        poly = make_poly_from_text(f"data/{i}.poly")
        parts = convex_partition(poly)
        plot(parts, color='r')
        plt.show()
# }}}

def test_color_triangulation():
    """ Draws a three-coloured triangulation of data/ polygons """
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
