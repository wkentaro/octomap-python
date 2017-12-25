#!/usr/bin/python
# coding: utf8
import numpy
import octomap

if __name__ == "__main__":
    tree = octomap.OcTree(0.1)
    tree.insertPointCloud(numpy.array([[1.0, 0.0 ,0.0],
                                       [0.0, 0.0, 1.0],
                                       [-1.0, 0.0, 0.0],
                                       [0.0, 0.0, -1.0]]),
                          numpy.array([0.0, 1.0, 0.0]))
    if tree.writeBinary("test.bt"):
        print("Create octree file.")
    else:
        print("Cannot create octree file.")
