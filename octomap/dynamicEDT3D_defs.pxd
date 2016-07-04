from libcpp cimport bool

cdef extern from "octomap/math/Vector3.h" namespace "octomath":
    cdef cppclass Vector3:
        Vector3() except +
        Vector3(float, float, float) except +

cdef extern from "octomap/octomap_types.h" namespace "octomap":
    ctypedef Vector3 point3d

cdef extern from "octomap/OcTreeKey.h" namespace "octomap":
    cdef cppclass OcTreeKey:
        OcTreeKey() except +
        OcTreeKey(unsigned short int a, unsigned short int b, unsigned short int c) except +

cdef extern from "octomap/OcTree.h" namespace "octomap":
    cdef cppclass OcTree

cdef extern from "dynamicEDT3D/dynamicEDTOctomap.h":
    cdef cppclass DynamicEDTOctomap:
        DynamicEDTOctomap(float maxdist, OcTree *_octree, point3d bbxMin, point3d bbxMax, bool treatUnknownAsOccupied)
        bool checkConsistency()
        float getDistance(point3d& p)
        float getDistance(OcTreeKey& k)
        float getMaxDist()
        void update(bool updateRealDist)
