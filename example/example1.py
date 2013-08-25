import numpy
import octomap

if __name__ == "__main__":
    tree = octomap.OcTree(0.1)
    tree.insertPointCloud(numpy.array([[1.0, 0.0 ,0.0],
                                       [-1.0, 0.0, 0.0]]),
                          numpy.array([0.0, 1.0, 0.0]))
    tree.writeBinary("test.bt")

