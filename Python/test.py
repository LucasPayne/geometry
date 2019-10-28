from shapes import *
from utils import *
from operations import *
from intersections import *
from plotting import *
from triangulation import *
from randomize import *
from random import random
import readline
import functools
from math import pi
from matplotlib import pyplot as plt

from test_algorithms import *
from test_collision import *
from test_bounding_volumes import *
from test_intersections import *
from test_misc import *

def probability_intersection():
    """ computes approx proportion of line segment
        pairs which intersect given their ends are
        uniformly distributed on a square. """
    num = 100000
    intersecting_count = 0
    for _ in range(num):
        seg1 = LineSeg(Point.random(1), Point.random(1))
        seg2 = LineSeg(Point.random(1), Point.random(1))
        if intersecting(seg1, seg2):
            intersecting_count += 1
    print(intersecting_count / num)

################################

def prefix_function(function, prefunction):
    # from SO: hook python module function
    @functools.wraps(function)
    def run(*args, **kwargs):
        prefunction(*args, **kwargs)
        return function(*args, **kwargs)
    return run

def prefix_plt_show():
    plt.gca().set_aspect('equal', adjustable='box')
plt.show = prefix_function(plt.show, prefix_plt_show)

if len(sys.argv) == 2:
    eval(f"test_{sys.argv[1]}()")
    sys.exit()

def complete(text, state):
    for cmd in commands:
        if cmd.startswith(text):
            if not state:
                return cmd
            else:
                state -= 1

commands = [name[len("test_"):] for name in dir() if name.startswith("test_")]
for command in commands:
    print(f"\t{command}")

readline.parse_and_bind("tab: complete")
readline.set_completer(complete)

while True:
    inp = input('Enter test name: ')
    if inp in commands:
        eval(f"test_{inp}()")
        break
