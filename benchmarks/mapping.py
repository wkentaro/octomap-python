"""Run with `uv run python benchmarks/mapping.py`; JSON goes to stdout."""

import argparse
import gc
import json
import platform
import statistics
import time
import tracemalloc

import numpy as np

import octomap


def legacy_labels(tree, points):
    labels = np.full(len(points), -1, dtype=np.int32)
    for i, point in enumerate(points):
        try:
            labels[i] = tree.isNodeOccupied(
                tree.search(tree.coordToKey(point))
            )
        except octomap.NullPointerException:
            pass
    return labels


def legacy_extract(tree):
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


def average_colors(tree, points, colors):
    for point, color in zip(points, colors):
        tree.averageNodeColor(
            point, int(color[0]), int(color[1]), int(color[2])
        )


def measure(operation, repeats):
    operation()
    samples = []
    for _ in range(repeats):
        gc.collect()
        start = time.perf_counter()
        operation()
        samples.append(time.perf_counter() - start)
    gc.collect()
    tracemalloc.start()
    operation()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return {
        "median_ms": statistics.median(samples) * 1000,
        "min_ms": min(samples) * 1000,
        "max_ms": max(samples) * 1000,
        "peak_traced_bytes": peak,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=int, default=30000)
    parser.add_argument("--queries", type=int, default=100000)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument(
        "--save", help="Save inputs and results to an NPZ file"
    )
    args = parser.parse_args()
    if min(args.points, args.queries, args.repeats) < 1:
        parser.error("counts must be positive")
    rng = np.random.default_rng(42)
    points = rng.uniform([-1, -1, 0.5], [1, 1, 1.5], (args.points, 3))
    origin = np.zeros(3)
    queries = rng.uniform([-2, -2, -0.5], [2, 2, 2], (args.queries, 3))
    colors = rng.integers(0, 255, (args.points, 3), dtype=np.uint8)
    output = {
        "python": platform.python_version(),
        "numpy": np.__version__,
        "platform": platform.platform(),
        "points": args.points,
        "queries": args.queries,
        "repeats": args.repeats,
        "resolution": 0.05,
        "results": {},
    }
    saved = {"points": points, "queries": queries, "colors": colors}
    for tree_class in (octomap.OcTree, octomap.ColorOcTree):

        def insert():
            new_tree = tree_class(0.05)
            new_tree.insertPointCloud(points, origin)
            return new_tree

        tree = insert()
        labels = getattr(tree, "getLabels", lambda p: legacy_labels(tree, p))
        extract = getattr(
            tree, "extractPointCloud", lambda: legacy_extract(tree)
        )
        np.testing.assert_array_equal(
            labels(queries[:64]), legacy_labels(tree, queries[:64])
        )
        result = {
            "insert": measure(insert, args.repeats),
            "labels": measure(lambda: labels(queries), args.repeats),
            "extract": measure(extract, args.repeats),
            "leaf_count": tree.getNumLeafNodes(),
            "tree_memory_bytes": tree.memoryUsage(),
        }
        prefix = tree_class.__name__
        saved[prefix + "_labels"] = labels(queries)
        occupied, free = extract()
        saved[prefix + "_occupied"] = occupied
        saved[prefix + "_free"] = free
        saved[prefix + "_tree"] = np.frombuffer(tree.write(), dtype=np.uint8)
        if tree_class is octomap.ColorOcTree:
            colorize = getattr(tree, "averageNodeColors", None)
            result["colors"] = measure(
                lambda: (
                    colorize(points, colors)
                    if colorize is not None
                    else average_colors(tree, points, colors)
                ),
                args.repeats,
            )
            saved[prefix + "_colored_tree"] = np.frombuffer(
                tree.write(), dtype=np.uint8
            )
        output["results"][prefix] = result
    if args.save:
        np.savez(args.save, allow_pickle=False, **saved)
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
