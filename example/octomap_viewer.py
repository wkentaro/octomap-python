#!/usr/bin/python
# coding: utf8
import sys
import numpy
import octomap
from mayavi.mlab import *

class OctomapViewer:
    def __init__(self, filename):
        self.tree = octomap.OcTree(filename)
    def draw(self):
        itr = self.tree.begin_tree()
        op = []
        fp = []
        so = []
        sf = []
        for i in itr:
            if i.isLeaf():
                if self.tree.isNodeOccupied(i):
                    so.append(i.getSize())
                    op.append(i.getCoordinate())
                else:
                    sf.append(i.getSize())
                    fp.append(i.getCoordinate())
        op = zip(*op)
        fp = zip(*fp)
        points3d(op[0], op[1], op[2], so, opacity=1.0, mode='cube', color=(0, 0, 1), scale_mode='none', scale_factor=0.1)
        points3d(fp[0], fp[1], fp[2], sf, opacity=0.3, mode='cube', color=(0, 1, 0), scale_mode='none', scale_factor=0.1)
        show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: python octomap_viewer.py [octomap filename]"
    else:
        viewer = OctomapViewer(sys.argv[1])
        viewer.draw()
