#!/usr/bin/env python

import imgviz
import numpy as np
from viewer import pointcloud_from_depth
from viewer import visualize

import octomap


def _build_color_octree(points, colors, resolution, maxrange):
    octree = octomap.ColorOcTree(resolution)
    octree.insertPointCloud(
        pointcloud=points,
        origin=np.array([0, 0, 0], dtype=float),
        maxrange=maxrange,
    )
    for point, color in zip(points, colors):
        octree.averageNodeColor(
            point, int(color[0]), int(color[1]), int(color[2])
        )
    octree.updateInnerOccupancy()
    return octree


def _extract_colored_voxels(octree, resolution):
    points = []
    colors = []
    for it in octree.begin_leafs():
        if not octree.isNodeOccupied(it):
            continue
        center = it.getCoordinate()
        # a pruned leaf can be larger than one voxel; expand it into
        # resolution-sized voxels that all share the leaf's color
        dimension = max(1, round(it.getSize() / resolution))
        origin = center - (dimension / 2 - 0.5) * resolution
        indices = np.indices((dimension, dimension, dimension))
        indices = indices.reshape(3, -1).T
        points.append(origin + indices * resolution)
        colors.append(np.tile(it.getColor(), (len(indices), 1)))

    if not points:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    return (
        np.concatenate(points, axis=0),
        np.concatenate(colors, axis=0).astype(np.uint8),
    )


def main():
    data = imgviz.data.arc2017()
    camera_info = data["camera_info"]
    K = np.array(camera_info["K"]).reshape(3, 3)
    rgb = data["rgb"]
    pcd = pointcloud_from_depth(
        data["depth"], fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
    )

    nonnan = ~np.isnan(pcd).any(axis=2)
    mask = nonnan & np.less(pcd[:, :, 2], 2)
    masked_pcd = pcd[mask]
    masked_rgb = rgb[mask]

    resolution = 0.01

    octree = octomap.OcTree(resolution)
    # The OcTree takes every valid point (maxrange clips the raycast at 2 m);
    # the ColorOcTree below uses the tighter z < 2 m mask so its per-point
    # color loop only visits points it actually inserted.
    octree.insertPointCloud(
        pointcloud=pcd[nonnan],
        origin=np.array([0, 0, 0], dtype=float),
        maxrange=2,
    )
    occupied, empty = octree.extractPointCloud()

    # Alternatively, classify a dense query grid with getLabels() instead of
    # reading the tree's leaves with extractPointCloud(). getLabels() returns
    # -1 (unknown / never observed), 0 (free), or 1 (occupied) for arbitrary
    # points -- the typical collision-query use:
    #
    # points = ...  # (N, 3) query points
    # labels = octree.getLabels(points)
    # occupied = points[labels == 1]
    # empty = points[labels == 0]

    color_octree = _build_color_octree(
        points=masked_pcd, colors=masked_rgb, resolution=resolution, maxrange=2
    )
    occupied_color = _extract_colored_voxels(color_octree, resolution)

    visualize(
        pointcloud=(masked_pcd, masked_rgb),
        occupied=occupied,
        occupied_color=occupied_color,
        empty=empty,
        K=K,
        width=camera_info["width"],
        height=camera_info["height"],
        resolution=resolution,
        aabb=(octree.getMetricMin(), octree.getMetricMax()),
    )


if __name__ == "__main__":
    main()
