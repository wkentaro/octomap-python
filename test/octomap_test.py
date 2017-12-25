#!/usr/bin/python
# coding: utf8
import octomap
import numpy as np
import unittest

class OctreeTestCase(unittest.TestCase):
    def setUp(self):
        self.tree = octomap.OcTree(0.1)
    def tearDown(self):
        pass

    def test_readBinary(self):
        self.assertTrue(self.tree.readBinary(b"test.bt"))
        self.assertFalse(self.tree.readBinary(b"test0.bt"))

    def test_writeBinary(self):
        self.assertTrue(self.tree.writeBinary(b"test1.bt"))

    def test_checkTree(self):
        self.tree.readBinary(b"test.bt")
        data = self.tree.write()
        tree2 = octomap.OcTree.read(data)
        self.assertEqual(tree2.write(), data)

    def test_checkBinary(self):
        self.tree.readBinary(b"test.bt")
        data = self.tree.writeBinary()
        tree2 = octomap.OcTree(0.1)
        tree2.readBinary(data)
        self.assertEqual(tree2.writeBinary(), data)

    def test_BBXMax(self):
        a = np.array((1.0, 2.0, 3.0))
        self.tree.setBBXMax(a)
        self.assertTrue(np.allclose(self.tree.getBBXMax(), a))

    def test_BBXMin(self):
        a = np.array((1.0, 2.0, 3.0))
        self.tree.setBBXMin(a)
        self.assertTrue(np.allclose(self.tree.getBBXMin(), a))

    def test_Resolution(self):
        r = 1.0
        self.tree.setResolution(r)
        self.assertAlmostEqual(self.tree.getResolution(), r)

    def test_Node(self):
        self.tree.insertPointCloud(np.array([[1.0, 0.0 ,0.0]]),
                                   np.array([0.0, 0.0, 0.0]))
        node = self.tree.getRoot()
        self.assertAlmostEqual(node.getValue(), 0.847298, places=5)
        self.assertEqual(node.childExists(0), False)
        self.assertEqual(node.childExists(1), False)
        self.assertEqual(node.childExists(2), False)
        self.assertEqual(node.childExists(3), False)
        self.assertEqual(node.childExists(4), False)
        self.assertEqual(node.childExists(5), False)
        self.assertEqual(node.childExists(6), False)
        self.assertEqual(node.childExists(7), True)
        self.assertAlmostEqual(self.tree.getNodeChild(node, 7).getValue(), 0.847298, places=5)

    def test_Update(self):
        test_point1 = np.array([1.0, 2.0, 3.0])
        test_point2 = np.array([0.0, 0.0, 0.0])
        test_point3 = np.array([5.0, 5.0, 5.0])
        self.tree.insertPointCloud(np.array([test_point1]),
                                   np.array([0.0, 0.0, 0.0]))
        node1 = self.tree.search(test_point1)
        node2 = self.tree.search(test_point2)
        node3 = self.tree.search(test_point3)
        self.assertTrue(self.tree.isNodeOccupied(node1))
        self.assertFalse(self.tree.isNodeOccupied(node2))
        self.assertRaises(octomap.NullPointerException, lambda : self.tree.isNodeOccupied(node3))

        self.tree.updateNode(test_point2, True)
        self.tree.updateInnerOccupancy()
        self.assertTrue(self.tree.isNodeOccupied(node2))

    def test_Iterator(self):
        self.tree.insertPointCloud(np.array([[1.0, 0.0 ,0.0],
                                             [0.0, 0.0, 1.0],
                                             [-1.0, 0.0, 0.0],
                                             [0.0, 0.0, -1.0]]),
                                   np.array([0.0, 1.0, 0.0]))
        nodes = [i for i in self.tree.begin_tree() if i.isLeaf()]
        leafs = [i for i in self.tree.begin_leafs()]
        leafs_bbx = [i for i in self.tree.begin_leafs_bbx(np.array([0.0, 0.0, 0.0]),
                                                          np.array([1.0, 0.0, 0.0]))]
        self.assertEqual(len(nodes), len(leafs))
        self.assertEqual(len(leafs_bbx), 2)

    def test_castRay(self):
        origin = np.array([0.0, 0.0, 0.0])
        direction = np.array([1.0, 0.0, 0.0])
        end = np.array([0.0, 0.0, 0.0])

        # miss
        hit = self.tree.castRay(
            origin=origin,
            direction=direction,
            end=end,
            ignoreUnknownCells=True,
        )
        self.assertFalse(hit)
        self.assertTrue(np.all(end == 0.0))

        self.tree.insertPointCloud(
            np.array([
                [1.0, 0.0 , 0.0],
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0]
            ]),
            np.array([0.0, 0.0, 0.0])
        )

        # hit
        hit = self.tree.castRay(
            origin=origin,
            direction=direction,
            end=end,
            ignoreUnknownCells=True,
        )
        self.assertTrue(hit)
        self.assertTrue(np.allclose(end, [1.05, 0.05, 0.05]))

    def test_updateNodes(self):
        # test points
        test_point1 = np.array([1.0, 2.0, 3.0])
        test_point2 = np.array([0.0, 0.0, 0.0])
        test_point3 = np.array([5.0, 5.0, 5.0])

        # *not* present
        node1 = self.tree.search(test_point1)
        node2 = self.tree.search(test_point2)
        node3 = self.tree.search(test_point3)
        self.assertRaises(octomap.NullPointerException, lambda : self.tree.isNodeOccupied(node1))
        self.assertRaises(octomap.NullPointerException, lambda : self.tree.isNodeOccupied(node2))
        self.assertRaises(octomap.NullPointerException, lambda : self.tree.isNodeOccupied(node3))

        # batch update w/ test points
        self.tree.updateNodes([test_point1, test_point2, test_point3], True)

        # occupied
        node1 = self.tree.search(test_point1)
        node2 = self.tree.search(test_point2)
        node3 = self.tree.search(test_point3)
        self.assertTrue(self.tree.isNodeOccupied(node1))
        self.assertTrue(self.tree.isNodeOccupied(node2))
        self.assertTrue(self.tree.isNodeOccupied(node3))

    def test_insertDiscretizedPointCloud(self):
        test_point1 = np.array([1.0, 2.0, 3.0])
        test_point2 = np.array([0.0, 0.0, 0.0])
        test_point3 = np.array([5.0, 5.0, 5.0])

        self.tree.insertPointCloud(np.array([test_point1]),
                                   np.array([0.0, 0.0, 0.0]),
                                   discretize=True)

        node1 = self.tree.search(test_point1)
        node2 = self.tree.search(test_point2)
        node3 = self.tree.search(test_point3)

        self.assertTrue(self.tree.isNodeOccupied(node1))
        self.assertFalse(self.tree.isNodeOccupied(node2))
        self.assertRaises(octomap.NullPointerException, lambda : self.tree.isNodeOccupied(node3))


if __name__ == "__main__":
    unittest.main()
