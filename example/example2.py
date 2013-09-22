import numpy
import octomap

if __name__ == "__main__":
    tree = octomap.OcTree(0.1)
    tree.insertPointCloud(numpy.array([[1.0, 0.0 ,0.0],
                                       [0.0, 0.0, 1.0],
                                       [-1.0, 0.0, 0.0],
                                       [0.0, 0.0, -1.0]]),
                          numpy.array([0.0, 1.0, 0.0]))
    itr = tree.begin_tree()
    for i in itr:
        print "Coordinate: ", i.getCoordinate()
        print "Size: ", i.getSize()
        print "Depth: ", i.getDepth()
        key = i.getKey()
        print "Key: ", key[0], key[1], key[2]
        print "IsLeaf: ", i.isLeaf()
    print "End Iteration."
    #print tree.end_tree().isLeaf()
