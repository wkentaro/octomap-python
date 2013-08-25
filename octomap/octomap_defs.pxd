import sys

from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "math/Vector3.h" namespace "octomath":
    cdef cppclass Vector3:
        Vector3(float, float, float) except +

cdef extern from "octomap_types.h" namespace "octomap":
    ctypedef Vector3 point3d

cdef extern from "Pointcloud.h" namespace "octomap":
    cdef cppclass Pointcloud:
        Pointcloud() except +
        void push_back(float, float, float)
        void push_back(point3d* p)
        
cdef extern from "OcTree.h" namespace "octomap":
    cdef cppclass OcTree:
        OcTree(double resolution) except +
        OcTree(string _filename) except +
        bool writeBinary(string filename)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)

