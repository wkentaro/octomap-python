#!/usr/bin/env python

import imgviz
import numpy as np
import trimesh
import trimesh.viewer

import octomap

from insertPointCloud import pointcloud_from_depth
from insertPointCloud import visualize


def main():
    data = imgviz.data.arc2017()
    camera_info = data['camera_info']
    K = np.array(camera_info['K']).reshape(3, 3)
    rgb = data['rgb']
    pcd = pointcloud_from_depth(
        data['depth'], fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
    )

    nonnan = ~np.isnan(pcd).any(axis=2)
    mask = np.less(pcd[:, :, 2], 2)

    resolution = 0.01
    octree = octomap.OcTree(resolution)
    octree.insertPointCloud(
        pointcloud=pcd[nonnan],
        origin=np.array([0, 0, 0], dtype=float),
        maxrange=1.25,
    )

    aabb_min = octree.getMetricMin()
    aabb_max = octree.getMetricMax()
    center = (aabb_min + aabb_max) / 2
    dimension = np.array([64, 64, 64])
    origin = center - dimension / 2 * resolution

    aabb_min = origin - resolution / 2
    aabb_max = origin + dimension * resolution + resolution / 2

    grid = np.full(dimension, -1, np.int32)
    transform = trimesh.transformations.scale_and_translate(
        scale=resolution, translate=origin
    )
    points = trimesh.voxel.VoxelGrid(encoding=grid, transform=transform).points
    labels = octree.getLabels(points)

    occupied = points[labels == 1]
    empty = points[labels == 0]

    visualize(
        occupied=occupied,
        empty=empty,
        K=K,
        width=camera_info['width'],
        height=camera_info['height'],
        rgb=rgb,
        pcd=pcd,
        mask=mask,
        resolution=resolution,
        aabb=(aabb_min, aabb_max),
    )


if __name__ == '__main__':
    main()
