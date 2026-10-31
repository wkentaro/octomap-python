import numpy as np
import pytest

import octomap


@pytest.fixture(params=[octomap.OcTree, octomap.ColorOcTree])
def tree(request):
    return request.param(0.1)


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
@pytest.mark.parametrize("layout", ["contiguous", "strided", "readonly"])
def test_batch_labels(tree, dtype, layout):
    points = np.array([[1, 0, 0], [0, 0, 0], [5, 5, 5]], dtype=dtype)
    tree.insertPointCloud(points[:1].astype(float), np.zeros(3))
    if layout == "strided":
        points = np.repeat(points, 2, axis=1)[::-1, ::2]
    elif layout == "readonly":
        points.setflags(write=False)
    expected = []
    for point in points.astype(float):
        try:
            expected.append(
                int(tree.isNodeOccupied(tree.search(tree.coordToKey(point))))
            )
        except octomap.NullPointerException:
            expected.append(-1)
    labels = tree.getLabels(points)
    np.testing.assert_array_equal(labels, expected)
    assert labels.dtype == np.int32
    assert tree.getLabels(np.empty((0, 3), dtype=dtype)).shape == (0,)


def test_batch_labels_invalid_coordinates(tree):
    tree.updateNode(np.zeros(3), True)
    points = np.array(
        [[np.nan, 0, 0], [np.inf, 0, 0], [1e20, 0, 0], [6553.6, 0, 0]]
    )
    np.testing.assert_array_equal(tree.getLabels(points), [-1, -1, -1, -1])
    with pytest.raises(ValueError, match="shape"):
        tree.getLabels(np.zeros((2, 2)))


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
@pytest.mark.parametrize("discretize", [False, True])
@pytest.mark.parametrize("lazy_eval", [False, True])
def test_batch_insert(tree, dtype, discretize, lazy_eval):
    rng = np.random.default_rng(3)
    points = rng.uniform(-1, 1, (50, 6)).astype(dtype)[::-1, ::2]
    points.setflags(write=False)
    origin = np.zeros(3, dtype=np.float32)
    expected = type(tree)(0.1)
    options = dict(maxrange=0.9, lazy_eval=lazy_eval, discretize=discretize)
    expected.insertPointCloud(
        points.astype(float), origin.astype(float), **options
    )
    tree.insertPointCloud(points, origin, **options)
    if lazy_eval:
        expected.updateInnerOccupancy()
        tree.updateInnerOccupancy()
    assert tree.write() == expected.write()


@pytest.mark.parametrize(
    "points, origin",
    [
        (np.zeros((2, 2)), np.zeros(3)),
        (np.zeros((2, 4)), np.zeros(3)),
        (np.zeros((2, 3)), np.zeros(2)),
        (np.zeros((2, 3)), np.zeros(4)),
        (np.array([[0.0, 0, 0], [np.nan, 0, 0]]), np.zeros(3)),
        (np.array([[np.inf, 0, 0]]), np.zeros(3)),
        (np.array([[1e20, 0, 0]]), np.zeros(3)),
        (np.array([[3276.8, 0, 0]]), np.zeros(3)),
        (np.zeros((2, 3)), np.array([1e20, 0, 0])),
        (np.zeros((2, 3)), np.array([np.nan, 0, 0])),
    ],
)
def test_batch_insert_rejects_invalid_before_mutation(tree, points, origin):
    before = tree.write()
    with pytest.raises(ValueError):
        tree.insertPointCloud(points, origin)
    assert tree.write() == before


def test_batch_insert_empty(tree):
    tree.insertPointCloud(np.empty((0, 3)), np.zeros(3))
    assert tree.size() == 0


def reference_extract(tree):
    resolution = float(np.float32(tree.getResolution()))
    occupied, free = [], []
    for leaf in tree.begin_leafs():
        dimension = max(1, round(leaf.getSize() / resolution))
        origin = leaf.getCoordinate() - (dimension / 2 - 0.5) * resolution
        indices = np.column_stack(
            np.nonzero(np.ones((dimension, dimension, dimension)))
        )
        points = origin + indices * resolution
        (occupied if tree.isNodeOccupied(leaf) else free).append(points)
    return tuple(
        np.concatenate(parts) if parts else np.empty((0, 3))
        for parts in (occupied, free)
    )


def test_extract_matches_leaf_expansion(tree):
    assert all(a.shape == (0, 3) for a in tree.extractPointCloud())
    points = np.indices((4, 4, 4)).reshape(3, -1).T * 0.1 + 0.05
    tree.updateNodes(points, True)
    tree.updateNodes(points + 1, False)
    tree.updateNode(np.array([-0.15, -0.25, -0.35]), True)
    assert any(
        leaf.getSize() > tree.getResolution() for leaf in tree.begin_leafs()
    )
    before = tree.write()
    for actual, expected in zip(
        tree.extractPointCloud(), reference_extract(tree)
    ):
        np.testing.assert_array_equal(actual, expected)
        assert actual.dtype == np.float64
    assert tree.write() == before
    tree.setOccupancyThres(0.9)
    for actual, expected in zip(
        tree.extractPointCloud(), reference_extract(tree)
    ):
        np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize("dtype", [np.float32, np.float64])
def test_average_colors_matches_ordered_scalar_updates(dtype):
    tree = octomap.ColorOcTree(0.1)
    expected = octomap.ColorOcTree(0.1)
    points = np.array([[0.05, 0, 0], [0.05, 0, 0], [1, 1, 1]], dtype=dtype)
    colors = np.array([[255, 0, 0], [0, 0, 255], [7, 8, 9]], dtype=np.uint8)
    tree.updateNode(points[0], True)
    expected.updateNode(points[0], True)
    points.setflags(write=False)
    colors.setflags(write=False)
    # Reversed views exercise strides. The missing node stays missing.
    tree.averageNodeColors(points[::-1], colors[::-1])
    for point, color in zip(points[::-1], colors[::-1]):
        expected.averageNodeColor(point, *map(int, color))
    tree.updateInnerOccupancy()
    expected.updateInnerOccupancy()
    assert tree.write() == expected.write()
    assert tree.search(points[0]).getColor() == (127, 0, 127)
    tree.averageNodeColors(
        np.empty((0, 3), dtype=dtype), np.empty((0, 3), dtype=np.uint8)
    )


def test_average_colors_validates_before_mutation():
    tree = octomap.ColorOcTree(0.1)
    points = np.zeros((2, 3))
    tree.updateNode(points[0], True)
    before = tree.write()
    with pytest.raises(ValueError, match="shape"):
        tree.averageNodeColors(points, np.zeros((1, 3), dtype=np.uint8))
    points[1, 0] = np.nan
    with pytest.raises(ValueError):
        tree.averageNodeColors(points, np.zeros((2, 3), dtype=np.uint8))
    assert tree.write() == before


@pytest.mark.parametrize("bad", [np.nan, np.inf, 1e20, 3276.8])
def test_average_colors_rejects_out_of_bounds_before_mutation(bad):
    tree = octomap.ColorOcTree(0.1)
    tree.updateNode(np.zeros(3), True)
    before = tree.write()
    with pytest.raises(ValueError):
        tree.averageNodeColors(
            np.array([[0.0, 0, 0], [bad, 0, 0]]),
            np.array([[255, 0, 0], [0, 0, 255]], dtype=np.uint8),
        )
    assert tree.write() == before
