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
        so = []
        sf = []
        for i in itr:
            if i.isLeaf():
                if self.tree.isNodeOccupied(i.node):
                    so.append(i.getSize())
                    op.append(i.getCoordinate())
                else:
                    if self.view_free:
                        sf.append(i.getSize())
                        fp.append(i.getCoordinate())
        op = zip(*op)
        fp = zip(*fp)
        points3d(op[0], op[1], op[2], so, opacity=1.0, mode='cube',
                 color=(0, 0, 1), scale_mode='none', scale_factor=0.1)
        if self.view_free:
            points3d(fp[0], fp[1], fp[2], sf, opacity=0.3, mode='cube',
                     color=(0, 1, 0), scale_mode='none', scale_factor=0.1)
        show()

if __name__ == "__main__":
    from optparse import OptionParser
    usage = "Usage: python %prog [options]"
    parser = OptionParser(usage)
    parser.add_option("-f", "--file", action="store",
                      type="string", dest="filename", help="Input file name")
    parser.add_option("-s", action="store_true",
                      default=False, dest="free", help="Draw free space.")
    (options, args) = parser.parse_args()
    if options.filename is None:
        parser.print_help()
        exit()
    viewer = OctomapViewer(options.filename, options.free)
    viewer.draw()
