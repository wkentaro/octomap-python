#!/usr/bin/python
# coding: utf8

#This is an example that inserts
import numpy
import octomap

filename = "test.bt"
tree = octomap.OcTree(filename)
itr = tree.begin_tree()

# octomap resolution
resolution = tree.getResolution()
# z-position of the floor
floor_position = 0.0
big_enough_octomap_update_value = 3

min_point = tree.getMetricMin()
max_point = tree.getMetricMax()
print("Bounding box is at:")
print(min_point)
print(max_point)
for i in numpy.arange(min_point[0], max_point[0], resolution):
    for j in numpy.arange(min_point[1], max_point[1], resolution):
        tree.updateNode(numpy.array([i, j, floor_position]), big_enough_octomap_update_value)
tree.updateInnerOccupancy()
# tree.prune()
tree.writeBinary("testOut.bt")
