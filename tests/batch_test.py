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
