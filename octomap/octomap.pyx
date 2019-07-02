from libcpp.string cimport string
from libcpp cimport bool as cppbool
from libc.string cimport memcpy
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport octomap_defs as defs
cimport dynamicEDT3D_defs as edt
import numpy as np
cimport numpy as np
ctypedef np.float64_t DOUBLE_t
ctypedef defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator* tree_iterator_ptr
ctypedef defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_iterator* leaf_iterator_ptr
ctypedef defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_bbx_iterator* leaf_bbx_iterator_ptr

class NullPointerException(Exception):
    """
    Null pointer exception
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
    """
    Nodes to be used in OcTree.
    They represent 3d occupancy grid cells. "value" stores their log-odds occupancy.
    """
    cdef defs.OcTreeNode *thisptr
    def __cinit__(self):
        pass
    def __dealloc__(self):
        pass
    def addValue(self, float p):
        """
        adds p to the node's logOdds value (with no boundary / threshold checking!)
        """
        if self.thisptr:
            self.thisptr.addValue(p)
        else:
            raise NullPointerException
    def childExists(self, unsigned int i):
        """
        Safe test to check of the i-th child exists,
        first tests if there are any children.
        """
        if self.thisptr:
            return self.thisptr.childExists(i)
        else:
            raise NullPointerException
    def getValue(self):
        if self.thisptr:
            return self.thisptr.getValue()
        else:
            raise NullPointerException
    def setValue(self, float v):
        if self.thisptr:
            self.thisptr.setValue(v)
        else:
            raise NullPointerException
    def getOccupancy(self):
        if self.thisptr:
            return self.thisptr.getOccupancy()
        else:
            raise NullPointerException
    def getLogOdds(self):
        if self.thisptr:
            return self.thisptr.getLogOdds()
        else:
            raise NullPointerException
    def setLogOdds(self, float l):
        if self.thisptr:
            self.thisptr.setLogOdds(l)
        else:
            raise NullPointerException
    def hasChildren(self):
        if self.thisptr:
            return self.thisptr.hasChildren()
        else:
            raise NullPointerException

cdef class iterator_base:
    """
    Iterator over the complete tree (inner nodes and leafs).
    """
    cdef defs.OcTree *treeptr
    cdef defs.OccupancyOcTreeBase[defs.OcTreeNode].iterator_base *thisptr
    def __cinit__(self):
        pass

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def __is_end(self):
        return deref(self.thisptr) == self.treeptr.end_tree()

    def __is_acceseable(self):
        if self.thisptr and self.treeptr:
            if not self.__is_end():
                return True
        return False

    def getCoordinate(self):
        """
        return the center coordinate of the current node
        """
        cdef defs.Vector3 pt
        if self.__is_acceseable():
            pt = self.thisptr.getCoordinate()
            return np.array((pt.x(), pt.y(), pt.z()))
        else:
            raise NullPointerException

    def getDepth(self):
        if self.__is_acceseable():
            return self.thisptr.getDepth()
        else:
            raise NullPointerException

    def getKey(self):
        """
        the OcTreeKey of the current node
        """
        if self.__is_acceseable():
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
        if self.__is_acceseable():
            key = OcTreeKey()
            key.thisptr[0][0] = self.thisptr.getIndexKey()[0]
            key.thisptr[0][1] = self.thisptr.getIndexKey()[1]
            key.thisptr[0][2] = self.thisptr.getIndexKey()[2]
            return key
        else:
            raise NullPointerException

    def getSize(self):
        if self.__is_acceseable():
            return self.thisptr.getSize()
        else:
            raise NullPointerException

    def getX(self):
        if self.__is_acceseable():
            return self.thisptr.getX()
        else:
            raise NullPointerException
    def getY(self):
        if self.__is_acceseable():
            return self.thisptr.getY()
        else:
            raise NullPointerException
    def getZ(self):
        if self.__is_acceseable():
            return self.thisptr.getZ()
        else:
            raise NullPointerException

    def getOccupancy(self):
        if self.__is_acceseable():
            return (<defs.OcTreeNode>deref(deref(self.thisptr))).getOccupancy()
        else:
            raise NullPointerException

    def getValue(self):
        if self.__is_acceseable():
            return (<defs.OcTreeNode>deref(deref(self.thisptr))).getValue()
        else:
            raise NullPointerException


cdef class tree_iterator(iterator_base):
    """
    Iterator over the complete tree (inner nodes and leafs).
    """
    def __cinit__(self):
        pass

    def next(self):
        if self.thisptr and self.treeptr:
            if not self.__is_end():
                inc(deref(defs.static_cast[tree_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self.__is_end():
                yield self
                if self.thisptr:
                    inc(deref(defs.static_cast[tree_iterator_ptr](self.thisptr)))
                else:
                    break
        else:
            raise NullPointerException

    def isLeaf(self):
        if self.__is_acceseable():
            return defs.static_cast[tree_iterator_ptr](self.thisptr).isLeaf()
        else:
            raise NullPointerException

cdef class leaf_iterator(iterator_base):
    """
    Iterator over the complete tree (leafs).
    """
    def __cinit__(self):
        pass

    def next(self):
        if self.thisptr and self.treeptr:
            if not self.__is_end():
                inc(deref(defs.static_cast[leaf_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self.__is_end():
                yield self
                if self.thisptr:
                    inc(deref(defs.static_cast[leaf_iterator_ptr](self.thisptr)))
                else:
                    break
        else:
            raise NullPointerException

cdef class leaf_bbx_iterator(iterator_base):
    """
    Iterator over the complete tree (leafs).
    """
    def __cinit__(self):
        pass

    def next(self):
        if self.thisptr and self.treeptr:
            if not self.__is_end():
                inc(deref(defs.static_cast[leaf_bbx_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self.__is_end():
                yield self
                if self.thisptr:
                    inc(deref(defs.static_cast[leaf_bbx_iterator_ptr](self.thisptr)))
                else:
                    break
        else:
            raise NullPointerException

def _octree_read(filename):
    """
    Read the file header, create the appropriate class and deserialize.
    This creates a new octree which you need to delete yourself.
    """
    cdef defs.istringstream iss
    cdef OcTree tree = OcTree(0.1)
    if filename.startswith(b"# Octomap OcTree file"):
        iss.str(string(<char*?>filename, len(filename)))
        del tree.thisptr
        tree.thisptr = <defs.OcTree*>tree.thisptr.read(<defs.istream&?>iss)
        return tree
    else:
        del tree.thisptr
        tree.thisptr = <defs.OcTree*>tree.thisptr.read(string(<char*?>filename))
        return tree

cdef class OcTree:
    """
    octomap main map data structure, stores 3D occupancy grid map in an OcTree.
    """
    cdef defs.OcTree *thisptr
    cdef edt.DynamicEDTOctomap *edtptr
    def __cinit__(self, arg):
        import numbers
        if isinstance(arg, numbers.Number):
            self.thisptr = new defs.OcTree(<double?>arg)
        else:
            self.thisptr = new defs.OcTree(string(<char*?>arg))

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
        if self.edtptr:
            del self.edtptr

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

    def coordToKey(self, np.ndarray[DOUBLE_t, ndim=1] coord, depth=None):
        cdef defs.OcTreeKey key
        if depth is None:
            key = self.thisptr.coordToKey(defs.point3d(coord[0],
                                                       coord[1],
                                                       coord[2]))
        else:
            key = self.thisptr.coordToKey(defs.point3d(coord[0],
                                                       coord[1],
                                                       coord[2]),
                                          <unsigned int?>depth)
        res = OcTreeKey()
        res[0] = key[0]
        res[1] = key[1]
        res[2] = key[2]
        return res

    def coordToKeyChecked(self, np.ndarray[DOUBLE_t, ndim=1] coord, depth=None):
        cdef defs.OcTreeKey key
        cdef cppbool chk
        if depth is None:
            chk = self.thisptr.coordToKeyChecked(defs.point3d(coord[0],
                                                              coord[1],
                                                              coord[2]),
                                                 key)
        else:
            chk = self.thisptr.coordToKeyChecked(defs.point3d(coord[0],
                                                              coord[1],
                                                              coord[2]),
                                                 <unsigned int?>depth,
                                                 key)
        if chk:
            res = OcTreeKey()
            res[0] = key[0]
            res[1] = key[1]
            res[2] = key[2]
            return chk, res
        else:
            return chk, None

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
        cdef defs.point3d e
        cdef cppbool hit
        hit = self.thisptr.castRay(
            defs.point3d(origin[0], origin[1], origin[2]),
            defs.point3d(direction[0], direction[1], direction[2]),
            e,
            bool(ignoreUnknownCells),
            <double?>maxRange
        )
        if hit:
            end[0:3] = e.x(), e.y(), e.z()
        return hit

    read = _octree_read

    def write(self, filename=None):
        """
        Write file header and complete tree to file/stream (serialization)
        """
        cdef defs.ostringstream oss
        if not filename is None:
            return self.thisptr.write(string(<char*?>filename))
        else:
            ret = self.thisptr.write(<defs.ostream&?>oss)
            if ret:
                return oss.str().c_str()[:oss.str().length()]
            else:
                return False

    def readBinary(self, filename):
        cdef defs.istringstream iss
        if filename.startswith(b"# Octomap OcTree binary file"):
            iss.str(string(<char*?>filename, len(filename)))
            return self.thisptr.readBinary(<defs.istream&?>iss)
        else:
            return self.thisptr.readBinary(string(<char*?>filename))

    def writeBinary(self, filename=None):
        cdef defs.ostringstream oss
        if not filename is None:
            return self.thisptr.writeBinary(string(<char*?>filename))
        else:
            ret = self.thisptr.writeBinary(<defs.ostream&?>oss)
            if ret:
                return oss.str().c_str()[:oss.str().length()]
            else:
                return False

    def isNodeOccupied(self, node):
        if isinstance(node, OcTreeNode):
            if (<OcTreeNode>node).thisptr:
                return self.thisptr.isNodeOccupied(deref((<OcTreeNode>node).thisptr))
            else:
                raise NullPointerException
        else:
            return self.thisptr.isNodeOccupied(<defs.OcTreeNode>deref(deref((<tree_iterator>node).thisptr)))

    def isNodeAtThreshold(self, node):
        if isinstance(node, OcTreeNode):
            if (<OcTreeNode>node).thisptr:
                return self.thisptr.isNodeAtThreshold(deref((<OcTreeNode>node).thisptr))
            else:
                raise NullPointerException
        else:
            return self.thisptr.isNodeAtThreshold(<defs.OcTreeNode>deref(deref((<tree_iterator>node).thisptr)))

    def getLabels(self, np.ndarray[DOUBLE_t, ndim=2] points):
        cdef int i
        cdef np.ndarray[DOUBLE_t, ndim=1] pt
        cdef OcTreeKey key
        cdef OcTreeNode node
        # -1: unknown, 0: empty, 1: occupied
        cdef np.ndarray[np.int32_t, ndim=1] labels = \
            np.full((points.shape[0],), -1, dtype=np.int32)
        for i, pt in enumerate(points):
            key = self.coordToKey(pt)
            node = self.search(key)
            try:
                labels[i] = self.isNodeOccupied(node)
            except NullPointerException:
                pass
        return labels

    def extractPointCloud(self):
        cdef float resolution = self.getResolution()

        cdef list occupied = []
        cdef list empty = []
        cdef leaf_iterator it
        cdef float size
        cdef int is_occupied
        cdef np.ndarray[DOUBLE_t, ndim=1] center
        cdef np.ndarray[DOUBLE_t, ndim=1] origin
        cdef np.ndarray[np.int64_t, ndim=2] indices
        cdef np.ndarray[DOUBLE_t, ndim=2] points
        cdef np.ndarray keep
        cdef int dimension
        for it in self.begin_leafs():
            is_occupied = self.isNodeOccupied(it)
            size = it.getSize()
            center = it.getCoordinate()

            dimension = max(1, round(it.getSize() / resolution))
            origin = center - (dimension / 2 - 0.5) * resolution
            indices = np.column_stack(np.nonzero(np.ones((dimension, dimension, dimension))))
            points = origin + indices * np.array(resolution)

            if is_occupied:
                occupied.append(points)
            else:
                empty.append(points)

        cdef np.ndarray[DOUBLE_t, ndim=2] occupied_arr
        cdef np.ndarray[DOUBLE_t, ndim=2] empty_arr
        if len(occupied) == 0:
            occupied_arr = np.zeros((0, 3), dtype=float)
        else:
            occupied_arr = np.concatenate(occupied, axis=0)
        if len(empty) == 0:
            empty_arr = np.zeros((0, 3), dtype=float)
        else:
            empty_arr = np.concatenate(empty, axis=0)
        return occupied_arr, empty_arr

    def insertPointCloud(self,
                         np.ndarray[DOUBLE_t, ndim=2] pointcloud,
                         np.ndarray[DOUBLE_t, ndim=1] origin,
                         maxrange=-1.,
                         lazy_eval=False,
                         discretize=False):
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
                                      bool(lazy_eval),
                                      bool(discretize))

    def begin_tree(self, maxDepth=0):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self.thisptr.begin_tree(maxDepth))
        itr.treeptr = self.thisptr
        return itr

    def begin_leafs(self, maxDepth=0):
        itr = leaf_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_iterator(self.thisptr.begin_leafs(maxDepth))
        itr.treeptr = self.thisptr
        return itr

    def begin_leafs_bbx(self, np.ndarray[DOUBLE_t, ndim=1] bbx_min, np.ndarray[DOUBLE_t, ndim=1] bbx_max, maxDepth=0):
        itr = leaf_bbx_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_bbx_iterator(self.thisptr.begin_leafs_bbx(defs.point3d(bbx_min[0], bbx_min[1], bbx_min[2]),
                                                                                                                   defs.point3d(bbx_max[0], bbx_max[1], bbx_max[2]),
                                                                                                                   maxDepth))
        itr.treeptr = self.thisptr
        return itr

    def end_tree(self):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self.thisptr.end_tree())
        itr.treeptr = self.thisptr
        return itr

    def end_leafs(self):
        itr = leaf_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_iterator(self.thisptr.end_leafs())
        itr.treeptr = self.thisptr
        return itr

    def end_leafs_bbx(self):
        itr = leaf_bbx_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_bbx_iterator(self.thisptr.end_leafs_bbx())
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

    def getRoot(self):
        node = OcTreeNode()
        node.thisptr = self.thisptr.getRoot()
        return node

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

    def search(self, value, depth=0):
        node = OcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self.thisptr.search(defs.OcTreeKey(<unsigned short int>value[0],
                                                              <unsigned short int>value[1],
                                                              <unsigned short int>value[2]),
                                               <unsigned int?>depth)
        else:
            node.thisptr = self.thisptr.search(<double>value[0],
                                               <double>value[1],
                                               <double>value[2],
                                               <unsigned int?>depth)
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

    def updateNodes(self, values, update, lazy_eval=False):
        """
        Integrate occupancy measurements and Manipulate log_odds value of voxel directly. 
        """
        if values is None or len(values) == 0:
            return
        if isinstance(values[0], OcTreeKey):
            if isinstance(update, bool):
                for v in values:
                    self.thisptr.updateNode(defs.OcTreeKey(<unsigned short int>v[0],
                                                           <unsigned short int>v[1],
                                                           <unsigned short int>v[2]),
                                            <cppbool>update,
                                            <cppbool?>lazy_eval)
            else:
                for v in values:
                    self.thisptr.updateNode(defs.OcTreeKey(<unsigned short int>v[0],
                                                           <unsigned short int>v[1],
                                                           <unsigned short int>v[2]),
                                            <float?>update,
                                            <cppbool?>lazy_eval)
        else:
            if isinstance(update, bool):
                for v in values:
                    self.thisptr.updateNode(<double?>v[0],
                                            <double?>v[1],
                                            <double?>v[2],
                                            <cppbool>update,
                                            <cppbool?>lazy_eval)
            else:
                for v in values:
                    self.thisptr.updateNode(<double?>v[0],
                                            <double?>v[1],
                                            <double?>v[2],
                                            <float?>update,
                                            <cppbool?>lazy_eval)

    def updateNode(self, value, update, lazy_eval=False):
        """
        Integrate occupancy measurement and Manipulate log_odds value of voxel directly. 
        """
        node = OcTreeNode()
        if isinstance(value, OcTreeKey):
            if isinstance(update, bool):
                node.thisptr = self.thisptr.updateNode(defs.OcTreeKey(<unsigned short int>value[0],
                                                                      <unsigned short int>value[1],
                                                                      <unsigned short int>value[2]),
                                                       <cppbool>update,
                                                       <cppbool?>lazy_eval)
            else:
                node.thisptr = self.thisptr.updateNode(defs.OcTreeKey(<unsigned short int>value[0],
                                                                      <unsigned short int>value[1],
                                                                      <unsigned short int>value[2]),
                                                       <float?>update,
                                                       <cppbool?>lazy_eval)
        else:
            if isinstance(update, bool):
                node.thisptr = self.thisptr.updateNode(<double?>value[0],
                                                       <double?>value[1],
                                                       <double?>value[2],
                                                       <cppbool>update,
                                                       <cppbool?>lazy_eval)
            else:
                node.thisptr = self.thisptr.updateNode(<double?>value[0],
                                                       <double?>value[1],
                                                       <double?>value[2],
                                                       <float?>update,
                                                       <cppbool?>lazy_eval)
        return node

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

    def getClampingThresMax(self):
        return self.thisptr.getClampingThresMax()

    def getClampingThresMaxLog(self):
        return self.thisptr.getClampingThresMaxLog()

    def getClampingThresMin(self):
        return self.thisptr.getClampingThresMin()

    def getClampingThresMinLog(self):
        return self.thisptr.getClampingThresMinLog()

    def getOccupancyThres(self):
        return self.thisptr.getOccupancyThres()

    def getOccupancyThresLog(self):
        return self.thisptr.getOccupancyThresLog()

    def getProbHit(self):
        return self.thisptr.getProbHit()

    def getProbHitLog(self):
        return self.thisptr.getProbHitLog()

    def getProbMiss(self):
        return self.thisptr.getProbMiss()

    def getProbMissLog(self):
        return self.thisptr.getProbMissLog()

    def setClampingThresMax(self, double thresProb):
        self.thisptr.setClampingThresMax(thresProb)

    def setClampingThresMin(self, double thresProb):
        self.thisptr.setClampingThresMin(thresProb)

    def setOccupancyThres(self, double prob):
        self.thisptr.setOccupancyThres(prob)

    def setProbHit(self, double prob):
        self.thisptr.setProbHit(prob)

    def setProbMiss(self, double prob):
        self.thisptr.setProbMiss(prob)

    def getMetricSize(self):
        cdef double x = 0
        cdef double y = 0
        cdef double z = 0
        self.thisptr.getMetricSize(x, y, z)
        return np.array([x, y, z], dtype=float)

    def getMetricMin(self):
        cdef double x = 0
        cdef double y = 0
        cdef double z = 0
        self.thisptr.getMetricMin(x, y, z)
        return np.array([x, y, z], dtype=float)

    def getMetricMax(self):
        cdef double x = 0
        cdef double y = 0
        cdef double z = 0
        self.thisptr.getMetricMax(x, y, z)
        return np.array([x, y, z], dtype=float)

    def expandNode(self, node):
        self.thisptr.expandNode((<OcTreeNode>node).thisptr)

    def createNodeChild(self, node, int idx):
        child = OcTreeNode()
        child.thisptr = self.thisptr.createNodeChild((<OcTreeNode>node).thisptr, idx)
        return child

    def getNodeChild(self, node, int idx):
        child = OcTreeNode()
        child.thisptr = self.thisptr.getNodeChild((<OcTreeNode>node).thisptr, idx)
        return child

    def isNodeCollapsible(self, node):
        return self.thisptr.isNodeCollapsible((<OcTreeNode>node).thisptr)

    def deleteNodeChild(self, node, int idx):
        self.thisptr.deleteNodeChild((<OcTreeNode>node).thisptr, idx)

    def pruneNode(self, node):
        return self.thisptr.pruneNode((<OcTreeNode>node).thisptr)

    def dynamicEDT_generate(self, maxdist,
                            np.ndarray[DOUBLE_t, ndim=1] bbx_min,
                            np.ndarray[DOUBLE_t, ndim=1] bbx_max,
                            treatUnknownAsOccupied=False):
        self.edtptr = new edt.DynamicEDTOctomap(<float?>maxdist,
                                                self.thisptr,
                                                defs.point3d(bbx_min[0], bbx_min[1], bbx_min[2]),
                                                defs.point3d(bbx_max[0], bbx_max[1], bbx_max[2]),
                                                <cppbool?>treatUnknownAsOccupied)

    def dynamicEDT_checkConsistency(self):
        if self.edtptr:
            return self.edtptr.checkConsistency()
        else:
            raise NullPointerException

    def dynamicEDT_update(self, updateRealDist):
        if self.edtptr:
            self.edtptr.update(<cppbool?>updateRealDist)
        else:
            raise NullPointerException

    def dynamicEDT_getMaxDist(self):
        if self.edtptr:
            return self.edtptr.getMaxDist()
        else:
            raise NullPointerException

    def dynamicEDT_getDistance(self, p):
        if self.edtptr:
            if isinstance(p, OcTreeKey):
                return self.edtptr.getDistance(edt.OcTreeKey(<unsigned short int>p[0],
                                                             <unsigned short int>p[1],
                                                             <unsigned short int>p[2]))
            else:
                return self.edtptr.getDistance(edt.point3d(<float?>p[0],
                                                           <float?>p[1],
                                                           <float?>p[2]))
        else:
            raise NullPointerException
