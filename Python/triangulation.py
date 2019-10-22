#
# Triangulation and partitioning
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
