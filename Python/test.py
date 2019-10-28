""" 
    Select and run a test collected from the files in the tests/ directory.
"""

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

import sys
sys.path.append("./tests/")
for test_file in [name for name in os.listdir("tests/") if name.startswith("test_") and name.endswith(".py")]:
    module_name = test_file[:-len(".py")]
    exec(f"from {module_name} import *")
TEST_FUNCTIONS = [name for name in dir() if name.startswith("test_")]

def main():
    """
        Usage:
            python3 test.py: readline completion for selecting a test to run.
            python3 test.py <test name>: run this test
    """
    if len(sys.argv) == 2:
        eval(f"test_{sys.argv[1]}()")
        sys.exit()

    # Make sure pyplot always has equal axis aspects
    def prefix_plt_show():
        plt.gca().set_aspect('equal', adjustable='box')
    plt.show = prefix_function(plt.show, prefix_plt_show)

    # Initialize readline completer
    def complete(text, state):
        for cmd in commands:
            if cmd.startswith(text):
                if not state:
                    return cmd
                else:
                    state -= 1
    global TEST_FUNCTIONS
    commands = [name[len("test_"):] for name in TEST_FUNCTIONS]
    for command in commands:
        print(f"\t{command}")
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)

    # readline until valid test is named, run the test, then exit.
    while True:
        inp = input('Enter test name: ')
        if inp in commands:
            eval(f"test_{inp}()")
            break
        elif inp == "":
            break


if __name__ == "__main__":
    main()
