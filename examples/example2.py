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
    print("OccupancyThres: ", tree.getOccupancyThres(), tree.getOccupancyThresLog())
    print("ClampingThresMax: ", tree.getClampingThresMax(), tree.getClampingThresMaxLog())
    print("ClampingThresMin: ", tree.getClampingThresMin(), tree.getClampingThresMinLog())
    print("ProbHit: ", tree.getProbHit(), tree.getProbHitLog())
    print("ProbMiss: ", tree.getProbMiss(), tree.getProbMissLog())

    for i in tree.begin_tree():
        print("Coordinate: ", i.getCoordinate())
        print("Size: ", i.getSize())
        print("Depth: ", i.getDepth())
        key = i.getKey()
        print("Key: ", key[0], key[1], key[2])
        print("IsLeaf: ", i.isLeaf())
        print("Value: ", i.getValue())
        print("Occupancy: ", i.getOccupancy())
        print("Occupied:", tree.isNodeOccupied(i))
        print("AtThreshold:", tree.isNodeAtThreshold(i))
    print("End Iteration.")
