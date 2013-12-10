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


if __name__ == "__main__":
    unittest.main()
