#!/usr/bin/python
# coding: utf8
import numpy
import octomap
from mayavi.mlab import *

class OctomapViewer:
    def __init__(self, filename, view_free=False):
        self.tree = octomap.OcTree(filename)
        self.view_free = view_free
    def draw(self):
        itr = self.tree.begin_tree()
        op = []
        fp = []
        for i in itr:
            if i.isLeaf():
                if self.tree.isNodeOccupied(i):
                    so = i.getSize()
                    op.append(i.getCoordinate())
                else:
                    if self.view_free:
                        sf = i.getSize()
                        fp.append(i.getCoordinate())
        op = zip(*op)
        fp = zip(*fp)
        points3d(op[0], op[1], op[2], opacity=1.0, mode='cube',
                 color=(0, 0, 1), scale_mode='none', scale_factor=so)
        if self.view_free:
            points3d(fp[0], fp[1], fp[2], opacity=0.3, mode='cube',
                     color=(0, 1, 0), scale_mode='none', scale_factor=sf)
        show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Octomap viewer using mayavi')
    parser.add_argument("-f", "--file", action="store",
                        type=str, dest="filename", help="Input file name")
    parser.add_argument("-s", action="store_true",
                        default=False, dest="free", help="Draw free space.")
    options = parser.parse_args()

    viewer = OctomapViewer(options.filename, options.free)
    viewer.draw()
