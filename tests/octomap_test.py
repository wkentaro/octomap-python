import pathlib

import numpy as np
import pytest

import octomap

TEST_BT = str(pathlib.Path(__file__).parent / "test.bt").encode()


@pytest.fixture
def tree() -> octomap.OcTree:
    return octomap.OcTree(0.1)


def test_readBinary(tree: octomap.OcTree) -> None:
    assert tree.readBinary(TEST_BT)
    assert not tree.readBinary(b"test0.bt")


def test_writeBinary(tree: octomap.OcTree, tmp_path: pathlib.Path) -> None:
    assert tree.writeBinary(str(tmp_path / "test1.bt").encode())


def test_str_paths(tree: octomap.OcTree, tmp_path: pathlib.Path) -> None:
    tree.updateNode(np.array([1.0, 2.0, 3.0]), True)
    tree.updateInnerOccupancy()
    bt_path = str(tmp_path / "tree.bt")
    ot_path = str(tmp_path / "tree.ot")

    # writeBinary / readBinary round-trip with str paths
    assert tree.writeBinary(bt_path)
    expected_bt = tree.writeBinary()
    tree_bin = octomap.OcTree(0.1)
    assert tree_bin.readBinary(bt_path)
    assert tree_bin.writeBinary() == expected_bt

    # the str constructor reads a .bt file without TypeError
    tree_ctor = octomap.OcTree(bt_path)
    assert tree_ctor.writeBinary() == expected_bt

    # write / read round-trip with str paths
    assert tree.write(ot_path)
    expected_ot = tree.write()
    tree_full = octomap.OcTree.read(ot_path)
    assert tree_full.write() == expected_ot


def test_path_objects(tree: octomap.OcTree, tmp_path: pathlib.Path) -> None:
    tree.updateNode(np.array([1.0, 2.0, 3.0]), True)
    tree.updateInnerOccupancy()
    bt_path = tmp_path / "tree.bt"
    ot_path = tmp_path / "tree.ot"

    # writeBinary / readBinary round-trip with pathlib.Path objects
    assert tree.writeBinary(bt_path)
    expected_bt = tree.writeBinary()
    tree_bin = octomap.OcTree(0.1)
    assert tree_bin.readBinary(bt_path)
    assert tree_bin.writeBinary() == expected_bt

    # the constructor reads a .bt file from a pathlib.Path
    tree_ctor = octomap.OcTree(bt_path)
    assert tree_ctor.writeBinary() == expected_bt

    # write / read round-trip with pathlib.Path objects
    assert tree.write(ot_path)
    expected_ot = tree.write()
    tree_full = octomap.OcTree.read(ot_path)
    assert tree_full.write() == expected_ot


def test_read_missing_path_raises(tmp_path: pathlib.Path) -> None:
    with pytest.raises(FileNotFoundError):
        octomap.OcTree.read(tmp_path / "nonexistent.ot")


def test_read_unreadable_file_raises(tmp_path: pathlib.Path) -> None:
    corrupt = tmp_path / "corrupt.ot"
    corrupt.write_bytes(b"not an octree")
    with pytest.raises(OSError, match="failed to read"):
        octomap.OcTree.read(corrupt)


def test_checkTree(tree: octomap.OcTree) -> None:
    tree.readBinary(TEST_BT)
    data = tree.write()
    tree2 = octomap.OcTree.read(data)
    assert tree2.write() == data


def test_checkBinary(tree: octomap.OcTree) -> None:
    tree.readBinary(TEST_BT)
    data = tree.writeBinary()
    tree2 = octomap.OcTree(0.1)
    tree2.readBinary(data)
    assert tree2.writeBinary() == data


def test_BBXMax(tree: octomap.OcTree) -> None:
    a = np.array((1.0, 2.0, 3.0))
    tree.setBBXMax(a)
    assert np.allclose(tree.getBBXMax(), a)


def test_BBXMin(tree: octomap.OcTree) -> None:
    a = np.array((1.0, 2.0, 3.0))
    tree.setBBXMin(a)
    assert np.allclose(tree.getBBXMin(), a)


def test_Resolution(tree: octomap.OcTree) -> None:
    r = 1.0
    tree.setResolution(r)
    assert tree.getResolution() == pytest.approx(r)


def test_adjustKeyAtDepth(tree: octomap.OcTree) -> None:
    key = tree.coordToKey(np.array([1.0, 2.0, 3.0]))

    # adjusting to the full tree depth leaves the key unchanged
    assert tree.adjustKeyAtDepth(key, tree.getTreeDepth()) == key

    # at a coarser depth the key snaps to that depth's voxel and is stable
    # under a second adjustment
    coarse_depth = tree.getTreeDepth() - 2
    coarse = tree.adjustKeyAtDepth(key, coarse_depth)
    assert isinstance(coarse, octomap.OcTreeKey)
    assert tree.adjustKeyAtDepth(coarse, coarse_depth) == coarse


def test_Node(tree: octomap.OcTree) -> None:
    tree.insertPointCloud(
        np.array([[1.0, 0.0, 0.0]]), np.array([0.0, 0.0, 0.0])
    )
    node = tree.getRoot()
    assert node.getValue() == pytest.approx(0.847298, abs=1e-5)
    assert node.childExists(0) is False
    assert node.childExists(1) is False
    assert node.childExists(2) is False
    assert node.childExists(3) is False
    assert node.childExists(4) is False
    assert node.childExists(5) is False
    assert node.childExists(6) is False
    assert node.childExists(7) is True
    assert tree.getNodeChild(node, 7).getValue() == pytest.approx(
        0.847298, abs=1e-5
    )


def test_Update(tree: octomap.OcTree) -> None:
    test_point1 = np.array([1.0, 2.0, 3.0])
    test_point2 = np.array([0.0, 0.0, 0.0])
    test_point3 = np.array([5.0, 5.0, 5.0])
    tree.insertPointCloud(np.array([test_point1]), np.array([0.0, 0.0, 0.0]))
    node1 = tree.search(test_point1)
    node2 = tree.search(test_point2)
    node3 = tree.search(test_point3)
    assert tree.isNodeOccupied(node1)
    assert not tree.isNodeOccupied(node2)
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(node3)

    tree.updateNode(test_point2, True)
    tree.updateInnerOccupancy()
    assert tree.isNodeOccupied(node2)


def test_Iterator(tree: octomap.OcTree) -> None:
    tree.insertPointCloud(
        np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
        ),
        np.array([0.0, 1.0, 0.0]),
    )
    nodes = [i for i in tree.begin_tree() if i.isLeaf()]
    leafs = [i for i in tree.begin_leafs()]
    leafs_bbx = [
        i
        for i in tree.begin_leafs_bbx(
            np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
        )
    ]
    assert len(nodes) == len(leafs)
    assert len(leafs_bbx) == 2


def test_castRay(tree: octomap.OcTree) -> None:
    origin = np.array([0.0, 0.0, 0.0])
    direction = np.array([1.0, 0.0, 0.0])
    end = np.array([0.0, 0.0, 0.0])

    # miss still reports where the ray stopped: the voxel entered when
    # the ray reached maxRange=5.0 along x, with y/z at the origin voxel
    # center (0.05 = half of the 0.1 resolution)
    hit = tree.castRay(
        origin=origin,
        direction=direction,
        end=end,
        ignoreUnknownCells=True,
        maxRange=5.0,
    )
    assert not hit
    assert np.allclose(end, [5.05, 0.05, 0.05])

    tree.insertPointCloud(
        np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
        ),
        np.array([0.0, 0.0, 0.0]),
    )

    # hit
    end[:] = [9.0, 9.0, 9.0]
    hit = tree.castRay(
        origin=origin,
        direction=direction,
        end=end,
        ignoreUnknownCells=True,
    )
    assert hit
    assert np.allclose(end, [1.05, 0.05, 0.05])

    # a ray that cannot be cast (origin outside the tree) leaves end
    # unchanged rather than overwriting it
    end[:] = [7.0, 7.0, 7.0]
    hit = tree.castRay(
        origin=np.array([1e6, 1e6, 1e6]),
        direction=direction,
        end=end,
        ignoreUnknownCells=True,
        maxRange=5.0,
    )
    assert not hit
    assert np.allclose(end, [7.0, 7.0, 7.0])


def test_updateNodes(tree: octomap.OcTree) -> None:
    test_point1 = np.array([1.0, 2.0, 3.0])
    test_point2 = np.array([0.0, 0.0, 0.0])
    test_point3 = np.array([5.0, 5.0, 5.0])

    # *not* present
    node1 = tree.search(test_point1)
    node2 = tree.search(test_point2)
    node3 = tree.search(test_point3)
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(node1)
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(node2)
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(node3)

    # batch update w/ test points
    tree.updateNodes([test_point1, test_point2, test_point3], True)

    # occupied
    node1 = tree.search(test_point1)
    node2 = tree.search(test_point2)
    node3 = tree.search(test_point3)
    assert tree.isNodeOccupied(node1)
    assert tree.isNodeOccupied(node2)
    assert tree.isNodeOccupied(node3)


def test_insertDiscretizedPointCloud(tree: octomap.OcTree) -> None:
    test_point1 = np.array([1.0, 2.0, 3.0])
    test_point2 = np.array([0.0, 0.0, 0.0])
    test_point3 = np.array([5.0, 5.0, 5.0])

    tree.insertPointCloud(
        np.array([test_point1]), np.array([0.0, 0.0, 0.0]), discretize=True
    )

    node1 = tree.search(test_point1)
    node2 = tree.search(test_point2)
    node3 = tree.search(test_point3)

    assert tree.isNodeOccupied(node1)
    assert not tree.isNodeOccupied(node2)
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(node3)


def test_dynamicEDT(tree: octomap.OcTree) -> None:
    # A single occupied voxel at the origin: the Euclidean distance transform
    # should read ~0 at the obstacle and grow linearly with distance away from
    # it.
    tree.updateNode(np.array([0.0, 0.0, 0.0]), True)
    tree.updateInnerOccupancy()
    tree.dynamicEDT_generate(
        maxdist=4.0,
        bbx_min=np.array([-2.0, -2.0, -2.0]),
        bbx_max=np.array([2.0, 2.0, 2.0]),
        treatUnknownAsOccupied=False,
    )
    tree.dynamicEDT_update(True)

    res = tree.getResolution()
    assert tree.dynamicEDT_getDistance(
        np.array([0.0, 0.0, 0.0])
    ) == pytest.approx(0.0, abs=res)
    assert tree.dynamicEDT_getDistance(
        np.array([1.0, 0.0, 0.0])
    ) == pytest.approx(1.0, abs=res)
    assert tree.dynamicEDT_getMaxDist() >= 4.0

    # the only obstacle is the voxel at the origin, so the closest obstacle to
    # any query point is ~[0, 0, 0], and the returned distance matches
    # dynamicEDT_getDistance for the same query
    query = np.array([1.0, 0.0, 0.0])
    distance, obstacle = tree.dynamicEDT_getDistanceAndClosestObstacle(query)
    assert distance == tree.dynamicEDT_getDistance(query)
    assert obstacle is not None
    assert np.allclose(obstacle, [0.0, 0.0, 0.0], atol=res)

    # a query outside the distance map reports -1.0 and no obstacle
    distance, obstacle = tree.dynamicEDT_getDistanceAndClosestObstacle(
        np.array([10.0, 10.0, 10.0])
    )
    assert distance == -1.0
    assert obstacle is None

    # a query inside the map but beyond maxdist has its distance capped at the
    # maximum and no recorded obstacle (fresh tree to keep this tree's distance
    # field intact)
    far_tree = octomap.OcTree(0.1)
    far_tree.updateNode(np.array([0.0, 0.0, 0.0]), True)
    far_tree.updateInnerOccupancy()
    far_tree.dynamicEDT_generate(
        maxdist=1.0,
        bbx_min=np.array([-3.0, -3.0, -3.0]),
        bbx_max=np.array([3.0, 3.0, 3.0]),
        treatUnknownAsOccupied=False,
    )
    far_tree.dynamicEDT_update(True)
    distance, obstacle = far_tree.dynamicEDT_getDistanceAndClosestObstacle(
        np.array([2.0, 0.0, 0.0])
    )
    assert distance == far_tree.dynamicEDT_getMaxDist()
    assert obstacle is None
