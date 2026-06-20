from libcpp.string cimport string
from libcpp cimport bool as cppbool
from libc.string cimport memcpy
from libc.math cimport NAN, isnan
from cython.operator cimport dereference as deref, preincrement as inc, address
cimport octomap_defs as defs
cimport dynamicEDT3D_defs as edt
import numbers
import os
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

cdef class ColorOcTreeNode(OcTreeNode):
    """
    Node to be used in ColorOcTree. Extends OcTreeNode with a per-voxel RGB
    color, exposed to Python as an (r, g, b) tuple of ints in [0, 255].
    """
    cdef defs.ColorOcTreeNode* _color_node(self):
        return <defs.ColorOcTreeNode*>self.thisptr
    def getColor(self):
        cdef defs.ColorOcTreeNodeColor c
        if self.thisptr:
            c = self._color_node().getColor()
            return (c.r, c.g, c.b)
        else:
            raise NullPointerException
    def setColor(self, int r, int g, int b):
        if self.thisptr:
            self._color_node().setColor(<unsigned char>r,
                                        <unsigned char>g,
                                        <unsigned char>b)
        else:
            raise NullPointerException
    def isColorSet(self):
        if self.thisptr:
            return self._color_node().isColorSet()
        else:
            raise NullPointerException
    def getAverageChildColor(self):
        cdef defs.ColorOcTreeNodeColor c
        if self.thisptr:
            c = self._color_node().getAverageChildColor()
            return (c.r, c.g, c.b)
        else:
            raise NullPointerException

cdef class iterator_base:
    """
    Iterator over the complete tree (inner nodes and leafs).
    """
    cdef defs.AbstractOccupancyOcTree *treeptr
    cdef defs.iterator_base *thisptr
    # set by the iterator factories on ColorOcTree so getColor() is only
    # honored where the node actually carries a color
    cdef bint _has_color
    def __cinit__(self):
        pass

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def _is_end(self):
        # every end iterator (tree / leafs / leafs_bbx) is a default-constructed
        # one with a null tree and empty stack, so comparing against a default
        # iterator detects the end without depending on the concrete tree type
        cdef defs.iterator_base end
        return deref(self.thisptr) == end

    def _is_acceseable(self):
        if self.thisptr and self.treeptr:
            if not self._is_end():
                return True
        return False

    def getCoordinate(self):
        """
        return the center coordinate of the current node
        """
        cdef defs.Vector3 pt
        if self._is_acceseable():
            pt = self.thisptr.getCoordinate()
            return np.array((pt.x(), pt.y(), pt.z()))
        else:
            raise NullPointerException

    def getDepth(self):
        if self._is_acceseable():
            return self.thisptr.getDepth()
        else:
            raise NullPointerException

    def getKey(self):
        """
        the OcTreeKey of the current node
        """
        if self._is_acceseable():
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
        if self._is_acceseable():
            key = OcTreeKey()
            key.thisptr[0][0] = self.thisptr.getIndexKey()[0]
            key.thisptr[0][1] = self.thisptr.getIndexKey()[1]
            key.thisptr[0][2] = self.thisptr.getIndexKey()[2]
            return key
        else:
            raise NullPointerException

    def getSize(self):
        if self._is_acceseable():
            return self.thisptr.getSize()
        else:
            raise NullPointerException

    def getX(self):
        if self._is_acceseable():
            return self.thisptr.getX()
        else:
            raise NullPointerException
    def getY(self):
        if self._is_acceseable():
            return self.thisptr.getY()
        else:
            raise NullPointerException
    def getZ(self):
        if self._is_acceseable():
            return self.thisptr.getZ()
        else:
            raise NullPointerException

    def getOccupancy(self):
        if self._is_acceseable():
            return (<defs.OcTreeNode>deref(deref(self.thisptr))).getOccupancy()
        else:
            raise NullPointerException

    def getValue(self):
        if self._is_acceseable():
            return (<defs.OcTreeNode>deref(deref(self.thisptr))).getValue()
        else:
            raise NullPointerException

    def getColor(self):
        """
        the (r, g, b) color of the current node (ColorOcTree iterators only)
        """
        cdef defs.ColorOcTreeNodeColor c
        if not self._has_color:
            raise TypeError("getColor() is only available on ColorOcTree iterators")
        if self._is_acceseable():
            c = (<defs.ColorOcTreeNode*>address(deref(deref(self.thisptr)))).getColor()
            return (c.r, c.g, c.b)
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
            if not self._is_end():
                inc(deref(defs.static_cast[tree_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self._is_end():
                yield self
                if self.thisptr:
                    inc(deref(defs.static_cast[tree_iterator_ptr](self.thisptr)))
                else:
                    break
        else:
            raise NullPointerException

    def isLeaf(self):
        if self._is_acceseable():
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
            if not self._is_end():
                inc(deref(defs.static_cast[leaf_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self._is_end():
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
            if not self._is_end():
                inc(deref(defs.static_cast[leaf_bbx_iterator_ptr](self.thisptr)))
                return self
            else:
                raise StopIteration
        else:
            raise NullPointerException

    def __iter__(self):
        if self.thisptr and self.treeptr:
            while not self._is_end():
                yield self
                if self.thisptr:
                    inc(deref(defs.static_cast[leaf_bbx_iterator_ptr](self.thisptr)))
                else:
                    break
        else:
            raise NullPointerException

cdef class OccupancyOcTreeBase:
    """
    Shared base for occupancy octrees (OcTree, ColorOcTree).

    Holds the C++ object through an AbstractOccupancyOcTree pointer and binds
    every method whose C++ declaration lives on the non-templated abstract
    bases, so subclasses do not need to redeclare them. Subclasses own
    construction and the node-typed (templated) surface.
    """
    cdef defs.AbstractOccupancyOcTree *thisptr

    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr

    def _make_node(self):
        # the wrapper for nodes this tree returns; ColorOcTree overrides it so
        # updateNode hands back a color-aware node
        return OcTreeNode()

    def clear(self):
        self.thisptr.clear()

    def write(self, filename=None):
        """
        Write file header and complete tree to file/stream (serialization)
        """
        cdef defs.ostringstream oss
        if not filename is None:
            filename = os.fsencode(filename)
            return self.thisptr.write(string(<char*?>filename))
        else:
            ret = self.thisptr.write(<defs.ostream&?>oss)
            if ret:
                return oss.str().c_str()[:oss.str().length()]
            else:
                return False

    def readBinary(self, filename):
        cdef defs.istringstream iss
        filename = os.fsencode(filename)
        if filename.startswith(b"# Octomap OcTree binary file"):
            iss.str(string(<char*?>filename, len(filename)))
            return self.thisptr.readBinary(<defs.istream&?>iss)
        else:
            return self.thisptr.readBinary(string(<char*?>filename))

    def writeBinary(self, filename=None):
        cdef defs.ostringstream oss
        if not filename is None:
            filename = os.fsencode(filename)
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

    def getResolution(self):
        return self.thisptr.getResolution()

    def getTreeType(self):
        return self.thisptr.getTreeType().c_str()

    def memoryUsage(self):
        return self.thisptr.memoryUsage()

    def memoryUsageNode(self):
        return self.thisptr.memoryUsageNode()

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
                    self.thisptr.updateNode(defs.point3d(<double?>v[0],
                                                         <double?>v[1],
                                                         <double?>v[2]),
                                            <cppbool>update,
                                            <cppbool?>lazy_eval)
            else:
                for v in values:
                    self.thisptr.updateNode(defs.point3d(<double?>v[0],
                                                         <double?>v[1],
                                                         <double?>v[2]),
                                            <float?>update,
                                            <cppbool?>lazy_eval)

    def updateNode(self, value, update, lazy_eval=False):
        """
        Integrate occupancy measurement and Manipulate log_odds value of voxel directly.
        """
        cdef OcTreeNode node = self._make_node()
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
                node.thisptr = self.thisptr.updateNode(defs.point3d(<double?>value[0],
                                                                    <double?>value[1],
                                                                    <double?>value[2]),
                                                       <cppbool>update,
                                                       <cppbool?>lazy_eval)
            else:
                node.thisptr = self.thisptr.updateNode(defs.point3d(<double?>value[0],
                                                                    <double?>value[1],
                                                                    <double?>value[2]),
                                                       <float?>update,
                                                       <cppbool?>lazy_eval)
        return node

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

def _raise_read_error(serialized, filename, type_name):
    if serialized:
        raise ValueError(f"failed to deserialize {type_name} from the given bytes")
    path = os.fsdecode(filename)
    os.stat(path)  # raises FileNotFoundError if the path is missing
    raise OSError(f"failed to read {type_name} from {path!r}")

def _octree_read(filename):
    """
    Deserialize an OcTree from a file path or from its serialized bytes.
    """
    cdef defs.istringstream iss
    cdef OcTree tree = OcTree(0.1)
    # read() is a static factory; call it through the placeholder while it is
    # still alive, then free the placeholder we are replacing
    cdef defs.OcTree* placeholder = <defs.OcTree*>tree.thisptr
    filename = os.fsencode(filename)
    serialized = filename.startswith(b"# Octomap OcTree file")
    if serialized:
        iss.str(string(<char*?>filename, len(filename)))
        tree.thisptr = <defs.OcTree*>placeholder.read(<defs.istream&?>iss)
    else:
        tree.thisptr = <defs.OcTree*>placeholder.read(string(<char*?>filename))
    del placeholder
    if tree.thisptr == NULL:
        _raise_read_error(serialized=serialized, filename=filename, type_name="OcTree")
    return tree

cdef class OcTree(OccupancyOcTreeBase):
    """
    octomap main map data structure, stores 3D occupancy grid map in an OcTree.
    """
    cdef edt.DynamicEDTOctomap *edtptr
    def __cinit__(self, arg):
        if isinstance(arg, numbers.Number):
            self.thisptr = new defs.OcTree(<double?>arg)
        else:
            arg = os.fsencode(arg)
            self.thisptr = new defs.OcTree(string(<char*?>arg))

    def __dealloc__(self):
        if self.edtptr:
            del self.edtptr

    cdef defs.OcTree* _tree(self):
        return <defs.OcTree*>self.thisptr

    def adjustKeyAtDepth(self, OcTreeKey key, depth):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        cdef defs.OcTreeKey key_out = self._tree().adjustKeyAtDepth(key_in, <int?>depth)
        res = OcTreeKey()
        res[0] = key_out[0]
        res[1] = key_out[1]
        res[2] = key_out[2]
        return res

    def bbxSet(self):
        return self._tree().bbxSet()

    def calcNumNodes(self):
        return self._tree().calcNumNodes()

    def coordToKey(self, np.ndarray[DOUBLE_t, ndim=1] coord, depth=None):
        cdef defs.OcTreeKey key
        if depth is None:
            key = self._tree().coordToKey(defs.point3d(coord[0],
                                                       coord[1],
                                                       coord[2]))
        else:
            key = self._tree().coordToKey(defs.point3d(coord[0],
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
            chk = self._tree().coordToKeyChecked(defs.point3d(coord[0],
                                                              coord[1],
                                                              coord[2]),
                                                 key)
        else:
            chk = self._tree().coordToKeyChecked(defs.point3d(coord[0],
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
        return self._tree().deleteNode(defs.point3d(value[0],
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

        `end` receives the center of the voxel where the ray stopped,
        whether or not an occupied cell was hit, so a miss (return False)
        still reports the stopping point. If the ray cannot be cast at all
        (origin outside the tree, or a zero direction), `end` is left
        unchanged.
        """
        # seed e from end so that, like the C++ point3d& reference, end is
        # preserved on the early-exit paths where castRay never writes it
        cdef defs.point3d e = defs.point3d(end[0], end[1], end[2])
        cdef cppbool hit
        hit = self._tree().castRay(
            defs.point3d(origin[0], origin[1], origin[2]),
            defs.point3d(direction[0], direction[1], direction[2]),
            e,
            bool(ignoreUnknownCells),
            <double?>maxRange
        )
        end[0:3] = e.x(), e.y(), e.z()
        return hit

    read = _octree_read

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

        self._tree().insertPointCloud(pc,
                                      defs.Vector3(<float>origin[0],
                                                   <float>origin[1],
                                                   <float>origin[2]),
                                      <double?>maxrange,
                                      bool(lazy_eval),
                                      bool(discretize))

    def begin_tree(self, maxDepth=0):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self._tree().begin_tree(maxDepth))
        itr.treeptr = self._tree()
        return itr

    def begin_leafs(self, maxDepth=0):
        itr = leaf_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_iterator(self._tree().begin_leafs(maxDepth))
        itr.treeptr = self._tree()
        return itr

    def begin_leafs_bbx(self, np.ndarray[DOUBLE_t, ndim=1] bbx_min, np.ndarray[DOUBLE_t, ndim=1] bbx_max, maxDepth=0):
        itr = leaf_bbx_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_bbx_iterator(self._tree().begin_leafs_bbx(defs.point3d(bbx_min[0], bbx_min[1], bbx_min[2]),
                                                                                                                   defs.point3d(bbx_max[0], bbx_max[1], bbx_max[2]),
                                                                                                                   maxDepth))
        itr.treeptr = self._tree()
        return itr

    def end_tree(self):
        itr = tree_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].tree_iterator(self._tree().end_tree())
        itr.treeptr = self._tree()
        return itr

    def end_leafs(self):
        itr = leaf_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_iterator(self._tree().end_leafs())
        itr.treeptr = self._tree()
        return itr

    def end_leafs_bbx(self):
        itr = leaf_bbx_iterator()
        itr.thisptr = new defs.OccupancyOcTreeBase[defs.OcTreeNode].leaf_bbx_iterator(self._tree().end_leafs_bbx())
        itr.treeptr = self._tree()
        return itr

    def getBBXBounds(self):
        cdef defs.point3d p = self._tree().getBBXBounds()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXCenter(self):
        cdef defs.point3d p = self._tree().getBBXCenter()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMax(self):
        cdef defs.point3d p = self._tree().getBBXMax()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMin(self):
        cdef defs.point3d p = self._tree().getBBXMin()
        return np.array((p.x(), p.y(), p.z()))

    def getRoot(self):
        node = OcTreeNode()
        node.thisptr = self._tree().getRoot()
        return node

    def getNumLeafNodes(self):
        return self._tree().getNumLeafNodes()

    def getTreeDepth(self):
        return self._tree().getTreeDepth()

    def inBBX(self, np.ndarray[DOUBLE_t, ndim=1] p):
        return self._tree().inBBX(defs.point3d(p[0], p[1], p[2]))

    def keyToCoord(self, OcTreeKey key, depth=None):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        cdef defs.point3d p = defs.point3d()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        if depth is None:
            p = self._tree().keyToCoord(key_in)
        else:
            p = self._tree().keyToCoord(key_in, <int?>depth)
        return np.array((p.x(), p.y(), p.z()))

    def memoryFullGrid(self):
        return self._tree().memoryFullGrid()

    def resetChangeDetection(self):
        """
        Reset the set of changed keys. Call this after you obtained all changed nodes.
        """
        self._tree().resetChangeDetection()

    def search(self, value, depth=0):
        node = OcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self._tree().search(defs.OcTreeKey(<unsigned short int>value[0],
                                                              <unsigned short int>value[1],
                                                              <unsigned short int>value[2]),
                                               <unsigned int?>depth)
        else:
            node.thisptr = self._tree().search(<double>value[0],
                                               <double>value[1],
                                               <double>value[2],
                                               <unsigned int?>depth)
        return node

    def setBBXMax(self, np.ndarray[DOUBLE_t, ndim=1] max):
        """
        sets the maximum for a query bounding box to use
        """
        self._tree().setBBXMax(defs.point3d(max[0], max[1], max[2]))

    def setBBXMin(self, np.ndarray[DOUBLE_t, ndim=1] min):
        """
        sets the minimum for a query bounding box to use
        """
        self._tree().setBBXMin(defs.point3d(min[0], min[1], min[2]))

    def updateInnerOccupancy(self):
        """
        Updates the occupancy of all inner nodes to reflect their children's occupancy.
        """
        self._tree().updateInnerOccupancy()

    def useBBXLimit(self, enable):
        """
        use or ignore BBX limit (default: ignore)
        """
        self._tree().useBBXLimit(bool(enable))

    def volume(self):
        return self._tree().volume()

    def expandNode(self, node):
        self._tree().expandNode((<OcTreeNode>node).thisptr)

    def createNodeChild(self, node, int idx):
        child = OcTreeNode()
        child.thisptr = self._tree().createNodeChild((<OcTreeNode>node).thisptr, idx)
        return child

    def getNodeChild(self, node, int idx):
        child = OcTreeNode()
        child.thisptr = self._tree().getNodeChild((<OcTreeNode>node).thisptr, idx)
        return child

    def isNodeCollapsible(self, node):
        return self._tree().isNodeCollapsible((<OcTreeNode>node).thisptr)

    def deleteNodeChild(self, node, int idx):
        self._tree().deleteNodeChild((<OcTreeNode>node).thisptr, idx)

    def pruneNode(self, node):
        return self._tree().pruneNode((<OcTreeNode>node).thisptr)

    def dynamicEDT_generate(self, maxdist,
                            np.ndarray[DOUBLE_t, ndim=1] bbx_min,
                            np.ndarray[DOUBLE_t, ndim=1] bbx_max,
                            treatUnknownAsOccupied=False):
        self.edtptr = new edt.DynamicEDTOctomap(<float?>maxdist,
                                                self._tree(),
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

    def dynamicEDT_getDistanceAndClosestObstacle(self, p):
        """
        Return (distance, closest_obstacle_xyz) for the query point p. The
        closest obstacle is None when the library reports none: when p is
        outside the distance map (distance -1.0) or no obstacle lies within
        the maximum distance.
        """
        cdef float distance
        cdef edt.point3d obstacle = edt.point3d(NAN, NAN, NAN)
        if self.edtptr:
            self.edtptr.getDistanceAndClosestObstacle(
                edt.point3d(<float?>p[0], <float?>p[1], <float?>p[2]),
                distance,
                obstacle)
            if isnan(obstacle.x()):
                return distance, None
            return distance, np.array((obstacle.x(), obstacle.y(), obstacle.z()))
        else:
            raise NullPointerException

def _color_octree_read(filename):
    """
    Deserialize a ColorOcTree from a file path or from its serialized bytes.
    """
    cdef defs.istringstream iss
    cdef ColorOcTree tree = ColorOcTree(0.1)
    # read() is a static factory; call it through the placeholder while it is
    # still alive, then free the placeholder we are replacing
    cdef defs.ColorOcTree* placeholder = <defs.ColorOcTree*>tree.thisptr
    filename = os.fsencode(filename)
    serialized = filename.startswith(b"# Octomap OcTree file")
    if serialized:
        iss.str(string(<char*?>filename, len(filename)))
        tree.thisptr = <defs.ColorOcTree*>placeholder.read(<defs.istream&?>iss)
    else:
        tree.thisptr = <defs.ColorOcTree*>placeholder.read(string(<char*?>filename))
    del placeholder
    if tree.thisptr == NULL:
        _raise_read_error(serialized=serialized, filename=filename, type_name="ColorOcTree")
    return tree

cdef class ColorOcTree(OccupancyOcTreeBase):
    """
    OctoMap data structure that additionally stores an RGB color per voxel.

    Shares the full occupancy/search/IO surface with OcTree through
    OccupancyOcTreeBase and adds the color-specific methods. Note that only the
    full map format (write / read, ".ot") serializes color; the binary format
    (writeBinary / readBinary, ".bt") stores occupancy only.
    """
    def __cinit__(self, double resolution):
        self.thisptr = new defs.ColorOcTree(resolution)

    cdef defs.ColorOcTree* _tree(self):
        return <defs.ColorOcTree*>self.thisptr

    def _make_node(self):
        return ColorOcTreeNode()

    def setNodeColor(self, value, int r, int g, int b):
        """
        Set the color of the node at the given key or coordinate, replacing any
        previous color, and return it.
        """
        node = ColorOcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self._tree().setNodeColor(defs.OcTreeKey(<unsigned short int>value[0],
                                                                    <unsigned short int>value[1],
                                                                    <unsigned short int>value[2]),
                                                     <unsigned char>r, <unsigned char>g, <unsigned char>b)
        else:
            node.thisptr = self._tree().setNodeColor(<float>value[0], <float>value[1], <float>value[2],
                                                     <unsigned char>r, <unsigned char>g, <unsigned char>b)
        return node

    def averageNodeColor(self, value, int r, int g, int b):
        """
        Integrate a color measurement at the given key or coordinate by
        averaging it with the previous color, and return the node.
        """
        node = ColorOcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self._tree().averageNodeColor(defs.OcTreeKey(<unsigned short int>value[0],
                                                                        <unsigned short int>value[1],
                                                                        <unsigned short int>value[2]),
                                                         <unsigned char>r, <unsigned char>g, <unsigned char>b)
        else:
            node.thisptr = self._tree().averageNodeColor(<float>value[0], <float>value[1], <float>value[2],
                                                         <unsigned char>r, <unsigned char>g, <unsigned char>b)
        return node

    def integrateNodeColor(self, value, int r, int g, int b):
        """
        Integrate a color measurement at the given key or coordinate, weighted
        by the node's occupancy, and return the node.
        """
        node = ColorOcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self._tree().integrateNodeColor(defs.OcTreeKey(<unsigned short int>value[0],
                                                                          <unsigned short int>value[1],
                                                                          <unsigned short int>value[2]),
                                                           <unsigned char>r, <unsigned char>g, <unsigned char>b)
        else:
            node.thisptr = self._tree().integrateNodeColor(<float>value[0], <float>value[1], <float>value[2],
                                                           <unsigned char>r, <unsigned char>g, <unsigned char>b)
        return node

    def updateInnerOccupancy(self):
        """
        Update the occupancy and color of all inner nodes to reflect their
        children.
        """
        self._tree().updateInnerOccupancy()

    read = _color_octree_read

    # The methods below mirror OcTree's templated surface. They live on
    # OccupancyOcTreeBase<ColorOcTreeNode> in C++ and are typed on the concrete
    # node, so they cannot dispatch through the shared AbstractOccupancyOcTree
    # base and are intentionally duplicated here (Cython cdef classes cannot be
    # generic over the node type).
    def adjustKeyAtDepth(self, OcTreeKey key, depth):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        cdef defs.OcTreeKey key_out = self._tree().adjustKeyAtDepth(key_in, <int?>depth)
        res = OcTreeKey()
        res[0] = key_out[0]
        res[1] = key_out[1]
        res[2] = key_out[2]
        return res

    def bbxSet(self):
        return self._tree().bbxSet()

    def calcNumNodes(self):
        return self._tree().calcNumNodes()

    def coordToKey(self, np.ndarray[DOUBLE_t, ndim=1] coord, depth=None):
        cdef defs.OcTreeKey key
        if depth is None:
            key = self._tree().coordToKey(defs.point3d(coord[0], coord[1], coord[2]))
        else:
            key = self._tree().coordToKey(defs.point3d(coord[0], coord[1], coord[2]),
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
            chk = self._tree().coordToKeyChecked(defs.point3d(coord[0], coord[1], coord[2]), key)
        else:
            chk = self._tree().coordToKeyChecked(defs.point3d(coord[0], coord[1], coord[2]),
                                                 <unsigned int?>depth, key)
        if chk:
            res = OcTreeKey()
            res[0] = key[0]
            res[1] = key[1]
            res[2] = key[2]
            return chk, res
        else:
            return chk, None

    def deleteNode(self, np.ndarray[DOUBLE_t, ndim=1] value, depth=1):
        return self._tree().deleteNode(defs.point3d(value[0], value[1], value[2]),
                                       <int?>depth)

    def castRay(self, np.ndarray[DOUBLE_t, ndim=1] origin,
                np.ndarray[DOUBLE_t, ndim=1] direction,
                np.ndarray[DOUBLE_t, ndim=1] end,
                ignoreUnknownCells=False,
                maxRange=-1.0):
        cdef defs.point3d e = defs.point3d(end[0], end[1], end[2])
        cdef cppbool hit
        hit = self._tree().castRay(
            defs.point3d(origin[0], origin[1], origin[2]),
            defs.point3d(direction[0], direction[1], direction[2]),
            e,
            bool(ignoreUnknownCells),
            <double?>maxRange
        )
        end[0:3] = e.x(), e.y(), e.z()
        return hit

    def insertPointCloud(self,
                         np.ndarray[DOUBLE_t, ndim=2] pointcloud,
                         np.ndarray[DOUBLE_t, ndim=1] origin,
                         maxrange=-1.,
                         lazy_eval=False,
                         discretize=False):
        cdef defs.Pointcloud pc = defs.Pointcloud()
        for p in pointcloud:
            pc.push_back(<float>p[0], <float>p[1], <float>p[2])
        self._tree().insertPointCloud(pc,
                                      defs.Vector3(<float>origin[0],
                                                   <float>origin[1],
                                                   <float>origin[2]),
                                      <double?>maxrange,
                                      bool(lazy_eval),
                                      bool(discretize))

    # The iterator factories reuse the shared tree_iterator / leaf_iterator /
    # leaf_bbx_iterator wrappers. OccupancyOcTreeBase<ColorOcTreeNode>'s
    # iterators have the same layout as the OcTreeNode ones (the node type only
    # changes a stored pointer's type, and iterators carry no vtable), so the
    # reinterpret to defs.iterator_base* is layout-safe. _has_color tags the
    # wrapper so getColor() is honored.
    def begin_tree(self, maxDepth=0):
        itr = tree_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].tree_iterator(self._tree().begin_tree(maxDepth))
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def begin_leafs(self, maxDepth=0):
        itr = leaf_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].leaf_iterator(self._tree().begin_leafs(maxDepth))
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def begin_leafs_bbx(self, np.ndarray[DOUBLE_t, ndim=1] bbx_min, np.ndarray[DOUBLE_t, ndim=1] bbx_max, maxDepth=0):
        itr = leaf_bbx_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].leaf_bbx_iterator(self._tree().begin_leafs_bbx(defs.point3d(bbx_min[0], bbx_min[1], bbx_min[2]),
                                                                                                                                              defs.point3d(bbx_max[0], bbx_max[1], bbx_max[2]),
                                                                                                                                              maxDepth))
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def end_tree(self):
        itr = tree_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].tree_iterator(self._tree().end_tree())
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def end_leafs(self):
        itr = leaf_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].leaf_iterator(self._tree().end_leafs())
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def end_leafs_bbx(self):
        itr = leaf_bbx_iterator()
        itr.thisptr = <defs.iterator_base*>new defs.OccupancyOcTreeBase[defs.ColorOcTreeNode].leaf_bbx_iterator(self._tree().end_leafs_bbx())
        itr.treeptr = self.thisptr
        itr._has_color = True
        return itr

    def getBBXBounds(self):
        cdef defs.point3d p = self._tree().getBBXBounds()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXCenter(self):
        cdef defs.point3d p = self._tree().getBBXCenter()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMax(self):
        cdef defs.point3d p = self._tree().getBBXMax()
        return np.array((p.x(), p.y(), p.z()))

    def getBBXMin(self):
        cdef defs.point3d p = self._tree().getBBXMin()
        return np.array((p.x(), p.y(), p.z()))

    def getRoot(self):
        node = ColorOcTreeNode()
        node.thisptr = self._tree().getRoot()
        return node

    def getNumLeafNodes(self):
        return self._tree().getNumLeafNodes()

    def getTreeDepth(self):
        return self._tree().getTreeDepth()

    def inBBX(self, np.ndarray[DOUBLE_t, ndim=1] p):
        return self._tree().inBBX(defs.point3d(p[0], p[1], p[2]))

    def keyToCoord(self, OcTreeKey key, depth=None):
        cdef defs.OcTreeKey key_in = defs.OcTreeKey()
        cdef defs.point3d p = defs.point3d()
        key_in[0] = key[0]
        key_in[1] = key[1]
        key_in[2] = key[2]
        if depth is None:
            p = self._tree().keyToCoord(key_in)
        else:
            p = self._tree().keyToCoord(key_in, <int?>depth)
        return np.array((p.x(), p.y(), p.z()))

    def memoryFullGrid(self):
        return self._tree().memoryFullGrid()

    def resetChangeDetection(self):
        self._tree().resetChangeDetection()

    def search(self, value, depth=0):
        node = ColorOcTreeNode()
        if isinstance(value, OcTreeKey):
            node.thisptr = self._tree().search(defs.OcTreeKey(<unsigned short int>value[0],
                                                              <unsigned short int>value[1],
                                                              <unsigned short int>value[2]),
                                               <unsigned int?>depth)
        else:
            node.thisptr = self._tree().search(<double>value[0],
                                               <double>value[1],
                                               <double>value[2],
                                               <unsigned int?>depth)
        return node

    def setBBXMax(self, np.ndarray[DOUBLE_t, ndim=1] max):
        self._tree().setBBXMax(defs.point3d(max[0], max[1], max[2]))

    def setBBXMin(self, np.ndarray[DOUBLE_t, ndim=1] min):
        self._tree().setBBXMin(defs.point3d(min[0], min[1], min[2]))

    def useBBXLimit(self, enable):
        self._tree().useBBXLimit(bool(enable))

    def volume(self):
        return self._tree().volume()

    def expandNode(self, node):
        self._tree().expandNode(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr)

    def createNodeChild(self, node, int idx):
        child = ColorOcTreeNode()
        child.thisptr = self._tree().createNodeChild(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr, idx)
        return child

    def getNodeChild(self, node, int idx):
        child = ColorOcTreeNode()
        child.thisptr = self._tree().getNodeChild(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr, idx)
        return child

    def isNodeCollapsible(self, node):
        return self._tree().isNodeCollapsible(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr)

    def deleteNodeChild(self, node, int idx):
        self._tree().deleteNodeChild(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr, idx)

    def pruneNode(self, node):
        return self._tree().pruneNode(<defs.ColorOcTreeNode*>(<OcTreeNode>node).thisptr)
