import pathlib

import numpy as np
import pytest

import octomap


@pytest.fixture
def tree() -> octomap.ColorOcTree:
    return octomap.ColorOcTree(0.1)


def test_construct(tree: octomap.ColorOcTree) -> None:
    assert tree.getTreeType() == b"ColorOcTree"
    assert tree.getResolution() == pytest.approx(0.1)
    assert tree.size() == 0


def test_occupancy_update_and_search(tree: octomap.ColorOcTree) -> None:
    occupied = np.array([1.0, 2.0, 3.0])
    free = np.array([0.0, 0.0, 0.0])
    missing = np.array([5.0, 5.0, 5.0])

    tree.insertPointCloud(np.array([occupied]), np.array([0.0, 0.0, 0.0]))

    assert tree.isNodeOccupied(tree.search(occupied))
    assert not tree.isNodeOccupied(tree.search(free))
    with pytest.raises(octomap.NullPointerException):
        tree.isNodeOccupied(tree.search(missing))


def test_color_set_and_get(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)

    # a node with no color measurement defaults to white and reports unset
    node = tree.search(point)
    assert node.getColor() == (255, 255, 255)
    assert node.isColorSet() is False

    returned = tree.setNodeColor(point, 10, 20, 30)
    assert returned.getColor() == (10, 20, 30)
    assert tree.search(point).getColor() == (10, 20, 30)
    assert tree.search(point).isColorSet() is True


def test_node_setColor(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)

    node = tree.search(point)
    node.setColor(7, 8, 9)
    assert tree.search(point).getColor() == (7, 8, 9)


def test_updateNode_returns_color_node(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    # the node handed back by updateNode is color-aware, so it can be colored
    # without a second lookup
    node = tree.updateNode(point, True)
    assert isinstance(node, octomap.ColorOcTreeNode)
    node.setColor(5, 6, 7)
    assert tree.search(point).getColor() == (5, 6, 7)


def test_updateNodes_batch(tree: octomap.ColorOcTree) -> None:
    points = [np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 0.0])]
    tree.updateNodes(points, True)
    for point in points:
        assert tree.isNodeOccupied(tree.search(point))


def test_color_set_by_key(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    key = tree.coordToKey(point)

    tree.setNodeColor(key, 1, 2, 3)
    assert tree.search(key).getColor() == (1, 2, 3)
    assert tree.search(point).getColor() == (1, 2, 3)


def test_averageNodeColor(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)

    # first measurement on an unset node stores the color as-is
    tree.averageNodeColor(point, 255, 0, 0)
    assert tree.search(point).getColor() == (255, 0, 0)

    # a second measurement averages per channel: (255 + 0) // 2 etc.
    tree.averageNodeColor(point, 0, 0, 255)
    assert tree.search(point).getColor() == (127, 0, 127)

    # the key overload is a separate code path from the coordinate one
    key = tree.coordToKey(point)
    tree.averageNodeColor(key, 1, 0, 1)
    assert tree.search(key).getColor() == (64, 0, 64)


def test_integrateNodeColor(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 0, 0, 0)

    node = tree.integrateNodeColor(point, 255, 255, 255)
    r, g, b = node.getColor()
    # integration moves the color toward the measurement without overshooting
    assert 0 < r <= 255
    assert (r, g, b) == tree.search(point).getColor()

    # the key overload reaches the same node
    key = tree.coordToKey(point)
    assert (
        tree.integrateNodeColor(key, 0, 0, 0).getColor()
        == tree.search(key).getColor()
    )


def test_getAverageChildColor(tree: octomap.ColorOcTree) -> None:
    point1 = np.array([1.0, 2.0, 3.0])
    point2 = np.array([-1.0, -2.0, -3.0])
    tree.updateNode(point1, True)
    tree.updateNode(point2, True)
    tree.setNodeColor(point1, 200, 100, 50)
    tree.setNodeColor(point2, 100, 200, 150)
    tree.updateInnerOccupancy()

    # descend from the root through the single-child chain to the lowest common
    # ancestor of the two leaves; its averaged child color is the per-channel
    # mean of the two leaf colors
    node = tree.getRoot()
    while sum(node.childExists(i) for i in range(8)) == 1:
        child_index = next(i for i in range(8) if node.childExists(i))
        node = tree.getNodeChild(node, child_index)
    assert node.getAverageChildColor() == (150, 150, 100)


def test_iterator_reaches_color(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 10, 20, 30)
    tree.updateInnerOccupancy()

    colors = [
        it.getColor() for it in tree.begin_leafs() if tree.isNodeOccupied(it)
    ]
    assert colors == [(10, 20, 30)]


def test_getColor_rejects_plain_octree_iterator() -> None:
    # getColor() on a colorless OcTree iterator would read meaningless bytes,
    # so it is rejected rather than returning a plausible-looking tuple
    plain = octomap.OcTree(0.1)
    plain.updateNode(np.array([1.0, 2.0, 3.0]), True)
    plain.updateInnerOccupancy()
    for it in plain.begin_leafs():
        with pytest.raises(TypeError):
            it.getColor()


def test_iterator_counts_match(tree: octomap.ColorOcTree) -> None:
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


def test_write_read_preserves_color(
    tree: octomap.ColorOcTree, tmp_path: pathlib.Path
) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 10, 20, 30)
    tree.updateInnerOccupancy()

    # full map format (.ot) serializes color; round-trip via a str path
    ot_path = str(tmp_path / "tree.ot")
    assert tree.write(ot_path)
    loaded = octomap.ColorOcTree.read(ot_path)
    assert loaded.getTreeType() == b"ColorOcTree"
    assert loaded.search(point).getColor() == (10, 20, 30)


def test_write_read_pathlib(
    tree: octomap.ColorOcTree, tmp_path: pathlib.Path
) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 40, 50, 60)
    tree.updateInnerOccupancy()

    ot_path = tmp_path / "tree.ot"
    assert tree.write(ot_path)
    loaded = octomap.ColorOcTree.read(ot_path)
    assert loaded.search(point).getColor() == (40, 50, 60)


def test_write_read_bytes(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 11, 22, 33)
    tree.updateInnerOccupancy()

    data = tree.write()
    loaded = octomap.ColorOcTree.read(data)
    assert loaded.search(point).getColor() == (11, 22, 33)
    assert loaded.write() == data


def test_key_coord_roundtrip(tree: octomap.ColorOcTree) -> None:
    point = np.array([1.0, 2.0, 3.0])
    key = tree.coordToKey(point)
    assert isinstance(key, octomap.OcTreeKey)
    coord = tree.keyToCoord(key)
    # coordinate snaps to the voxel center, within half a resolution
    assert np.allclose(coord, point, atol=tree.getResolution())

    chk, checked = tree.coordToKeyChecked(point)
    assert chk
    assert checked == key


def test_shared_surface(tree: octomap.ColorOcTree) -> None:
    tree.updateNode(np.array([1.0, 2.0, 3.0]), True)
    tree.updateInnerOccupancy()

    assert tree.getTreeDepth() == 16
    assert tree.getNumLeafNodes() >= 1
    assert tree.size() > 0
    assert tree.volume() > 0

    metric_min = tree.getMetricMin()
    metric_max = tree.getMetricMax()
    assert np.all(metric_min <= metric_max)

    tree.clear()
    assert tree.size() == 0


def test_writeBinary_keeps_occupancy_drops_color(
    tree: octomap.ColorOcTree, tmp_path: pathlib.Path
) -> None:
    point = np.array([1.0, 2.0, 3.0])
    tree.updateNode(point, True)
    tree.setNodeColor(point, 10, 20, 30)
    tree.updateInnerOccupancy()

    bt_path = str(tmp_path / "tree.bt")
    assert tree.writeBinary(bt_path)

    loaded = octomap.ColorOcTree(0.1)
    assert loaded.readBinary(bt_path)
    # the binary format carries occupancy only, so color resets to unset
    assert loaded.isNodeOccupied(loaded.search(point))
    assert loaded.search(point).isColorSet() is False
