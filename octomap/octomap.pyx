from libcpp.string cimport string
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport octomap_defs as defs
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t

class NullPointerException(Exception):
    """
    Null pointer 例外
    """
    def __init__(self):
        pass

cdef class OcTreeKey:
    """
    OcTreeKey is a container class for internal key addressing.
    The keys count the number of cells (voxels) from the origin as discrete address of a voxel.
    """
    cdef defs.OcTreeKey *thisptr
    def __cinit__(self):
        self.thisptr = new defs.OcTreeKey()
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    def __setitem__(self, key, value):
        self.thisptr[0][key] = value
    def __getitem__(self, key):
        return self.thisptr[0][key]
    def __richcmp__(self, other, int op):
        if op == 2:
            return (self.thisptr[0][0] == other[0] and \
                    self.thisptr[0][1] == other[1] and \
                    self.thisptr[0][2] == other[2])
        elif op == 3:
            return not (self.thisptr[0][0] == other[0] and \
                        self.thisptr[0][1] == other[1] and \
                        self.thisptr[0][2] == other[2])

cdef class OcTreeNode:
    cdef defs.OcTreeNode *thisptr
    def __cinit__(self):
        self.thisptr = new defs.OcTreeNode()
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    def addValue(self, float p):
        self.thisptr.addValue(p)

    def getOccupancy(self):
        return self.thisptr.getOccupancy()

    def getValue(self):
        return self.thisptr.getValue()


cdef class tree_iterator:
    """
    Iterator over the complete tree (inner nodes and leafs).
    """
    cdef defs.OcTree *treeptr
    cdef defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator *thisptr
    cdef OcTreeNode node
    def __cinit__(self):
        pass

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def next(self):
        if self.thisptr:
            if deref(self.thisptr) != self.treeptr.end_tree():
                inc(deref(self.thisptr))
                self.node.thisptr[0] = <defs.OcTreeNode>deref(deref(self.thisptr))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while deref(self.thisptr) != self.treeptr.end_tree():
                self.node.thisptr[0] = <defs.OcTreeNode>deref(deref(self.thisptr))
                yield self
                if self.thisptr:
                    inc(deref(self.thisptr))
                else:
                    break
        else:
            raise NullPointerException

    def isLeaf(self):
        if self.thisptr:
            return self.thisptr.isLeaf()
        else:
            raise NullPointerException

    def getCoordinate(self):
        """
        return the center coordinate of the current node
        """
        cdef defs.Vector3 pt
        if self.thisptr:
            pt = self.thisptr.getCoordinate()
            return np.array((pt.x(), pt.y(), pt.z()))
        else:
            raise NullPointerException

    def getDepth(self):
        if self.thisptr:
            return self.thisptr.getDepth()
        else:
            raise NullPointerException

    def getKey(self):
        """
        the OcTreeKey of the current node
        """
        if self.thisptr:
            key = OcTreeKey()
            key.thisptr[0][0] = self.thisptr.getKey()[0]
            key.thisptr[0][1] = self.thisptr.getKey()[1]
            key.thisptr[0][2] = self.thisptr.getKey()[2]
            return key
        else:
            raise NullPointerException

    def getIndexKey(self):
        """
        the OcTreeKey of the current node, for nodes with depth != maxDepth
        """
        if self.thisptr:
            key = OcTreeKey()
            key.thisptr[0][0] = self.thisptr.getIndexKey()[0]
            key.thisptr[0][1] = self.thisptr.getIndexKey()[1]
            key.thisptr[0][2] = self.thisptr.getIndexKey()[2]
            return key
        else:
            raise NullPointerException

    def getSize(self):
        if self.thisptr:
            return self.thisptr.getSize()
        else:
            raise NullPointerException

    def getX(self):
        if self.thisptr:
            return self.thisptr.getX()
        else:
            raise NullPointerException
    def getY(self):
        if self.thisptr:
            return self.thisptr.getY()
        else:
            raise NullPointerException
    def getZ(self):
        if self.thisptr:
            return self.thisptr.getZ()
        else:
            raise NullPointerException

cdef class OcTree:
    """
    octomap main map data structure, stores 3D occupancy grid map in an OcTree.
    """
    cdef defs.OcTree *thisptr
    def __cinit__(self, arg):
        import numbers
        if isinstance(arg, numbers.Number):
            self.thisptr = new defs.OcTree(<double?>arg)
        else:
            self.thisptr = new defs.OcTree(string(<char*>arg))

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def adjustKeyAtDepth(self, OcTreeKey key, depth):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        cdef defs.OcTreeKey key_out = self.thisptr.adjustKeyAtDepth(key_in, <int?>depth)
        res = OcTreeKey
        res[0] = key_out[0]
        res[1] = key_out[1]
        res[2] = key_out[2]
        return res

    def bbxSet(self):
        return self.thisptr.bbxSet()

    def calcNumNodes(self):
        return self.thisptr.calcNumNodes()

    def clear(self):
        self.thisptr.clear()

    def coordToKey(self, np.ndarray[DOUBLE_t, ndim=1] coord):
        cdef defs.OcTreeKey key = self.thisptr.coordToKey(defs.point3d(coord[0],
                                                                       coord[1],
                                                                       coord[2]))
        res = OcTreeKey()
        res[0] = key[0]
        res[1] = key[1]
        res[2] = key[2]
        return res

    def deleteNode(self, np.ndarray[DOUBLE_t, ndim=1] value, depth=1):
        return self.thisptr.deleteNode(defs.point3d(value[0],
                                                    value[1],
                                                    value[2]),
                                       <int?>depth)

    def castRay(self, np.ndarray[DOUBLE_t, ndim=1] origin,
                np.ndarray[DOUBLE_t, ndim=1] direction,
                np.ndarray[DOUBLE_t, ndim=1] end,
                ignoreUnknownCells=False,
                maxRange=-1.0):
        """
        A ray is cast from origin with a given direction,
        the first occupied cell is returned (as center coordinate).
        If the starting coordinate is already occupied in the tree,
        this coordinate will be returned as a hit.
        """
        return self.thisptr.castRay(defs.point3d(origin[0], origin[1], origin[2]),
                                    defs.point3d(direction[0], direction[1], direction[2]),
                                    defs.point3d(end[0], end[1], end[2]),
                                    bool(ignoreUnknownCells),
                                    <double?>maxRange)

    def readBinary(self, char* filename):
        return self.thisptr.readBinary(string(filename))

    def writeBinary(self, char* filename):
        return self.thisptr.writeBinary(string(filename))

    def isNodeOccupied(self, OcTreeNode node):
        return self.thisptr.isNodeOccupied(deref(node.thisptr))

    def isNodeAtThreshold(self, OcTreeNode node):
        return self.thisptr.isNodeAtThreshold(deref(node.thisptr))

    def insertPointCloud(self,
                         np.ndarray[DOUBLE_t, ndim=2] pointcloud,
                         np.ndarray[DOUBLE_t, ndim=1] origin,
                         maxrange=-1.,
                         lazy_eval=False):
        """
        Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.

        Special care is taken that each voxel in the map is updated only once, and occupied
        nodes have a preference over free ones. This avoids holes in the floor from mutual
        deletion.
        :param pointcloud: Pointcloud (measurement endpoints), in global reference frame
        :param origin: measurement origin in global reference frame
        :param maxrange: maximum range for how long individual beams are inserted (default -1: complete beam)
        :param : whether update of inner nodes is omitted after the update (default: false).
        This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
        """
        cdef defs.Pointcloud pc = defs.Pointcloud()
        for p in pointcloud:
            pc.push_back(<float>p[0],
                         <float>p[1],
                         <float>p[2])

        self.thisptr.insertPointCloud(pc,
                                      defs.Vector3(<float>origin[0],
                                                   <float>origin[1],
                                                   <float>origin[2]),
                                      <double?>maxrange,
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

    def getBBXBounds(self):
        cdef defs.point3d p = self.thisptr.getBBXBounds()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXCenter(self):
        cdef defs.point3d p = self.thisptr.getBBXCenter()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMax(self):
        cdef defs.point3d p = self.thisptr.getBBXMax()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMin(self):
        cdef defs.point3d p = self.thisptr.getBBXMin()
        return np.array((p.x(), p.y(), p.z()))

    def getNumLeafNodes(self):
        return self.thisptr.getNumLeafNodes()

    def getResolution(self):
        return self.thisptr.getResolution()

    def getTreeDepth(self):
        return self.thisptr.getTreeDepth()

    def getTreeType(self):
        return self.thisptr.getTreeType().c_str()

    def inBBX(self, np.ndarray[DOUBLE_t, ndim=1] p):
        return self.thisptr.inBBX(defs.point3d(p[0], p[1], p[2]))

    def keyToCoord(self, OcTreeKey key, depth=None):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        cdef defs.point3d p = defs.point3d()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        if depth is None:
            p = self.thisptr.keyToCoord(key_in)
        else:
            p = self.thisptr.keyToCoord(key_in, <int?>depth)
        return np.array((p.x(), p.y(), p.z()))

    def memoryFullGrid(self):
        return self.thisptr.memoryFullGrid()

    def memoryUsage(self):
        return self.thisptr.memoryUsage()

    def memoryUsageNode(self):
        return self.thisptr.memoryUsageNode()

    def resetChangeDetection(self):
        """
        Reset the set of changed keys. Call this after you obtained all changed nodes.
        """
        self.thisptr.resetChangeDetection()

    def search(self, np.ndarray[DOUBLE_t, ndim=1] value, depth=0):
        node = OcTreeNode()
        node.thisptr[0] = self.thisptr.search(defs.point3d(value[0], value[1], value[2]),
                                              <unsigned int?>depth)[0]
        return node

    def setBBXMax(self, np.ndarray[DOUBLE_t, ndim=1] max):
        """
        sets the maximum for a query bounding box to use
        """
        self.thisptr.setBBXMax(defs.point3d(max[0], max[1], max[2]))

    def setBBXMin(self, np.ndarray[DOUBLE_t, ndim=1] min):
        """
        sets the minimum for a query bounding box to use
        """
        self.thisptr.setBBXMin(defs.point3d(min[0], min[1], min[2]))

    def setResolution(self, double r):
        """
        Change the resolution of the octree, scaling all voxels. This will not preserve the (metric) scale!
        """
        self.thisptr.setResolution(r)

    def size(self):
        return self.thisptr.size()

    def toMaxLikelihood(self):
        """
        Creates the maximum likelihood map by calling toMaxLikelihood on all tree nodes,
        setting their occupancy to the corresponding occupancy thresholds.
        """
        self.thisptr.toMaxLikelihood()

    def updateInnerOccupancy(self):
        """
        Updates the occupancy of all inner nodes to reflect their children's occupancy.
        """
        self.thisptr.updateInnerOccupancy()

    def useBBXLimit(self, enable):
        """
        use or ignore BBX limit (default: ignore)
        """
        self.thisptr.useBBXLimit(bool(enable))

    def volume(self):
        return self.thisptr.volume()

