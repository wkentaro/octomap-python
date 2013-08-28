from libcpp.string cimport string
cimport octomap_defs as defs
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

cdef class OcTree:
    """
    octomap main map data structure, stores 3D occupancy grid map in an OcTree.
    """
    cdef defs.OcTree *thisptr
    def __cinit__(self, arg):
        import numbers
        if isinstance(arg, numbers.Number):
            self.thisptr = new defs.OcTree(<float>arg)
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
