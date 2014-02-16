import octomap
import numpy as np
import unittest

class OctreeTestCase(unittest.TestCase):
    def setUp(self):
        self.tree = octomap.OcTree(0.1)
    def tearDown(self):
        pass

    def test_readBinary(self):
        self.assertTrue(self.tree.readBinary("test.bt"))
        self.assertFalse(self.tree.readBinary("test0.bt"))

    def test_writeBinary(self):
        self.assertTrue(self.tree.writeBinary("test1.bt"))

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
        self.assertAlmostEqual(node.getChild(7).getValue(), 0.847298, places=5)

if __name__ == "__main__":
    unittest.main()
