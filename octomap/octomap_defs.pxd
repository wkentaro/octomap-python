from libcpp cimport bool
from libcpp.string cimport string

cdef extern from "<iostream>" namespace "std":
    cdef cppclass istream:
        istream() except +
    cdef cppclass ostream:
        ostream() except +

cdef extern from "<sstream>" namespace "std":
    cdef cppclass istringstream:
        istringstream() except +
        istringstream(string& s) except +
        string str()
        void str(string& s)
    cdef cppclass ostringstream:
        ostringstream() except +
        string str()

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
        bool createChild(unsigned int i)
        void addValue(float& p)
        bool childExists(unsigned int i)
        float getValue()
        void setValue(float v)
        double getOccupancy()
        void expandNode()
        OcTreeNode* getChild(unsigned int i)
        float getLogOdds()
        void setLogOdds(float l)
        bool hasChildren()
        bool collapsible()
        void deleteChild(unsigned int i)
        bool pruneNode()

cdef extern from "octomap/OcTreeKey.h" namespace "octomap":
    cdef cppclass OcTreeKey:
        OcTreeKey() except +
        OcTreeKey(unsigned short int a, unsigned short int b, unsigned short int c) except +
        OcTreeKey(OcTreeKey& other)
        unsigned short int& operator[](unsigned int i)

cdef extern from "include_and_setting.h" namespace "octomap":
    cdef cppclass OccupancyOcTreeBase[T]:
        cppclass tree_iterator:
            tree_iterator() except +
            tree_iterator(tree_iterator&) except +
            point3d getCoordinate()
            unsigned int getDepth()
            OcTreeKey getIndexKey()
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

cdef extern from "include_and_setting.h" namespace "octomap":
    cdef cppclass OcTree:
        OcTree(double resolution) except +
        OcTree(string _filename) except +
        OcTreeKey adjustKeyAtDepth(OcTreeKey& key, unsigned int depth)
        unsigned short int adjustKeyAtDepth(unsigned short int key, unsigned int depth)
        bool bbxSet()
        size_t calcNumNodes()
        void clear()
        OcTreeKey coordToKey(point3d& coord)
        bool deleteNode(point3d& value, unsigned int depth)
        bool castRay(point3d& origin, point3d& direction, point3d& end,
                     bool ignoreUnknownCells, double maxRange)
        OcTree* read(string& filename)
        OcTree* read(istream& s)
        bool write(string& filename)
        bool write(ostream& s)
        bool readBinary(string& filename)
        bool readBinary(istream& s)
        bool writeBinary(string& filename)
        bool writeBinary(ostream& s)
        bool isNodeOccupied(OcTreeNode& occupancyNode)
        bool isNodeAtThreshold(OcTreeNode& occupancyNode)
        void insertPointCloud(Pointcloud& scan, point3d& sensor_origin,
                              double maxrange, bool lazy_eval)
        OccupancyOcTreeBase[OcTreeNode].tree_iterator begin_tree(unsigned char maxDepth) except +
        OccupancyOcTreeBase[OcTreeNode].tree_iterator end_tree() except +
        point3d getBBXBounds()
        point3d getBBXCenter()
        point3d getBBXMax()
        point3d getBBXMin()
        OcTreeNode* getRoot()
        size_t getNumLeafNodes()
        double getResolution()
        unsigned int getTreeDepth()
        string getTreeType()
        bool inBBX(point3d& p)
        point3d keyToCoord(OcTreeKey& key)
        point3d keyToCoord(OcTreeKey& key, unsigned int depth)
        unsigned long long memoryFullGrid()
        size_t memoryUsage()
        size_t memoryUsageNode()
        void resetChangeDetection()
        OcTreeNode* search(double x, double y, double z, unsigned int depth)
        OcTreeNode* search(point3d& value, unsigned int depth)
        OcTreeNode* search(OcTreeKey& key, unsigned int depth)
        void setBBXMax(point3d& max)
        void setBBXMin(point3d& min)
        void setResolution(double r)
        size_t size()
        void toMaxLikelihood()
        OcTreeNode* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval)
        OcTreeNode* updateNode(double x, double y, double z, bool occupied, bool lazy_eval)
        OcTreeNode* updateNode(OcTreeKey& key, float log_odds_update, bool lazy_eval)
        OcTreeNode* updateNode(OcTreeKey& key, bool occupied, bool lazy_eval)
        void updateInnerOccupancy()
        void useBBXLimit(bool enable)
        double volume()

        double getClampingThresMax()
        float getClampingThresMaxLog()
        double getClampingThresMin()
        float getClampingThresMinLog()

        double getOccupancyThres()
        float getOccupancyThresLog()
        double getProbHit()
        float getProbHitLog()
        double getProbMiss()
        float getProbMissLog()

        void setClampingThresMax(double thresProb)
        void setClampingThresMin(double thresProb)
        void setOccupancyThres(double prob)
        void setProbHit(double prob)
        void setProbMiss(double prob)
