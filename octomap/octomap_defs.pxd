from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "octomap/math/Vector3.h" namespace "octomath":
    cdef cppclass Vector3:
        Vector3() except +
        Vector3(float, float, float) except +
        Vector3(Vector3& other) except +
        float& x()
        float& y()
        float& z()

cdef extern from "octomap/octomap_types.h" namespace "octomap":
    ctypedef Vector3 point3d

cdef extern from "octomap/Pointcloud.h" namespace "octomap":
    cdef cppclass Pointcloud:
        Pointcloud() except +
        void push_back(float, float, float)
        void push_back(point3d* p)

cdef extern from "octomap/OcTreeNode.h" namespace "octomap":
    cdef cppclass OcTreeNode:
        OcTreeNode() except +
        float getValue()
        double getOccupancy()

cdef extern from "octomap/OcTreeKey.h" namespace "octomap":
    cdef cppclass OcTreeKey:
        OcTreeKey() except +
        OcTreeKey(OcTreeKey& other)
        unsigned short int& operator[](unsigned int i)

cdef extern from "octomap/OccupancyOcTreeBase.h" namespace "octomap":
    cdef cppclass OccupancyOcTreeBase[T]:
        cppclass tree_iterator:
            tree_iterator() except +
            tree_iterator(tree_iterator&) except +
            point3d getCoordinate()
            unsigned int getDepth()
            OcTreeKey& getKey()
            tree_iterator& operator++()
            OcTreeNode& operator*()
            bool operator==(tree_iterator &other)
            bool operator!=(tree_iterator &other)
            bool isLeaf() except +
            double getSize() except +
            double getX() except +
            double getY() except +
            double getZ() except +
        OccupancyOcTreeBase(double resolution) except +
        OccupancyOcTreeBase(string _filename) except +
        bool readBinary(string& filename)
        bool writeBinary(string& filename)
        bool isNodeOccupied(OcTreeNode& occupancyNode)
        bool isNodeAtThreshold(OcTreeNode& occupancyNode)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)
        OccupancyOcTreeBase[OcTreeNode].tree_iterator begin_tree(unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].tree_iterator end_tree() except +

cdef extern from "octomap/OcTree.h" namespace "octomap":
    cdef cppclass OcTree:
        OcTree(double resolution) except +
        OcTree(string _filename) except +
        bool readBinary(string& filename)
        bool writeBinary(string& filename)
        bool isNodeOccupied(OcTreeNode& occupancyNode)
        bool isNodeAtThreshold(OcTreeNode& occupancyNode)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)
        OccupancyOcTreeBase[OcTreeNode].tree_iterator begin_tree(unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].tree_iterator end_tree() except +