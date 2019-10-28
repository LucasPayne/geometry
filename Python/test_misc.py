from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *
from randomize import *

def test_triangle_base_height():
    """ Base-height box for triangle """
# {{{
    while True:
        tri = Triangle(Point.random(1), Point.random(1), Point.random(1))
        v = tri.B - tri.A
        n = normal_perp(v)
        height = dot(tri.C - tri.A, n)
        plot(tri, color='k')
        plot(LineSeg(tri.A, tri.B), color='r')
        plot(LineSeg(tri.A, tri.A + height * n), color='r')
        plot(LineSeg(tri.A + height * n, tri.B + height * n), color='r')
        plot(LineSeg(tri.B, tri.B + height * n), color='r')
        plt.show()
# }}}
