from libcpp.string cimport string
from cython.operator cimport dereference as deref, preincrement as inc
cimport octomap_defs as defs
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

cdef class tree_iterator:
    cdef defs.OcTree *treeptr
    cdef defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator *thisptr
    def __cinit__(self):
        pass

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def next(self):
        inc(deref(self.thisptr))
        return self

    def __iter__(self):
        while deref(self.thisptr) != self.treeptr.end_tree():
            yield self
            inc(deref(self.thisptr))

    def isLeaf(self):
        return self.thisptr.isLeaf()

    def getCoordinate(self):
        cdef defs.Vector3 pt = self.thisptr.getCoordinate()
        return [pt.x(), pt.y(), pt.z()]

    def getSize(self):
        return self.thisptr.getSize()

    def getX(self):
        return self.thisptr.getX()
    def getY(self):
        return self.thisptr.getY()
    def getZ(self):
        return self.thisptr.getZ()


cdef class OcTree:
    """
    octomap main map data structure, stores 3D occupancy grid map in an OcTree.
    """
    cdef defs.OcTree *thisptr
    def __cinit__(self, arg):
        import numbers
        if isinstance(arg, numbers.Number):
            self.thisptr = new defs.OcTree(<double>arg)
        else:
            self.thisptr = new defs.OcTree(string(<char*>arg))

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def readBinary(self, char* filename):
        return self.thisptr.readBinary(string(filename))

    def writeBinary(self, char* filename):
        return self.thisptr.writeBinary(string(filename))

    def insertPointCloud(self,
                         np.ndarray[DOUBLE_t, ndim=2] pointcloud,
                         np.ndarray[DOUBLE_t, ndim=1] origin,
                         maxrange=-1.,
                         lazy_eval=False):
        cdef defs.Pointcloud pc = defs.Pointcloud()
        for n in range(pointcloud.shape[0]):
            pc.push_back(<float>pointcloud[n, 0],
                         <float>pointcloud[n, 1],
                         <float>pointcloud[n, 2])

        self.thisptr.insertPointCloud(pc,
                                      defs.Vector3(<float>origin[0],
                                                   <float>origin[1],
                                                   <float>origin[2]),
                                      <double>maxrange,
                                      bool(lazy_eval))

    def begin_tree(self, maxDepth=0):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self.thisptr.begin_tree(maxDepth))
        itr.treeptr = self.thisptr
        return itr

    def end_tree(self):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self.thisptr.end_tree())
        itr.treeptr = self.thisptr
        return itr
