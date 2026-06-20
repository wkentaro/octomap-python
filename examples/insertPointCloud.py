#!/usr/bin/env python

import imgviz
import numpy as np
import trimesh
import trimesh.transformations as tf

import octomap


def pointcloud_from_depth(depth, fx, fy, cx, cy):
    assert depth.dtype.kind == "f", "depth must be float and have meter values"

    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    pc = np.dstack((x, y, z))

    return pc


def show_views(views, K, width, height, aabb):
    # glooey's labeled panes were dropped (its pyglet-1/ast.Str stack is stuck
    # on Python <=3.11); lay the views out side by side in a single scene
    # instead, each framed by the same bbox and sensor marker and offset along
    # X by 1.5x the bbox width.
    camera = trimesh.scene.Camera(
        resolution=(width, height), focal=(K[0, 0], K[1, 1])
    )
    camera_marker = trimesh.creation.camera_marker(camera, marker_height=0.1)
    # trimesh's camera_marker opens toward -Z (OpenGL convention) but the
    # depth point cloud uses OpenCV (+Z forward). Rotate the marker 180 deg
    # about X so the frustum opens toward the data instead of away from it.
    opencv_from_opengl = tf.rotation_matrix(np.pi, [1, 0, 0])
    for geometry in camera_marker:
        geometry.apply_transform(opencv_from_opengl)

    aabb_min, aabb_max = aabb
    bbox = trimesh.path.creation.box_outline(
        aabb_max - aabb_min,
        tf.translation_matrix((aabb_min + aabb_max) / 2),
    )

    step = (aabb_max[0] - aabb_min[0]) * 1.5
    scene = trimesh.Scene(camera=camera)
    for index, view in enumerate(views):
        offset = tf.translation_matrix([index * step, 0, 0])
        for geometry in [bbox, view, *camera_marker]:
            geometry = geometry.copy()
            geometry.apply_transform(offset)
            scene.add_geometry(geometry=geometry)

    scene.show(resolution=(int(640 * 0.9 * len(views)), int(480 * 0.9)))


def visualize(
    occupied, empty, K, width, height, rgb, pcd, mask, resolution, aabb
):
    pointcloud = trimesh.PointCloud(vertices=pcd[mask], colors=rgb[mask])
    occupied_boxes = trimesh.voxel.ops.multibox(
        occupied, pitch=resolution, colors=[1.0, 0, 0, 0.5]
    )
    empty_boxes = trimesh.voxel.ops.multibox(
        empty, pitch=resolution, colors=[0.5, 0.5, 0.5, 0.5]
    )
    show_views(
        views=[pointcloud, occupied_boxes, empty_boxes],
        K=K,
        width=width,
        height=height,
        aabb=aabb,
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
    mask = np.less(pcd[:, :, 2], 2)

    resolution = 0.01
    octree = octomap.OcTree(resolution)
    octree.insertPointCloud(
        pointcloud=pcd[nonnan],
        origin=np.array([0, 0, 0], dtype=float),
        maxrange=2,
    )
    occupied, empty = octree.extractPointCloud()

    aabb_min = octree.getMetricMin()
    aabb_max = octree.getMetricMax()

    # Alternatively, classify a dense query grid with getLabels() instead of
    # reading the tree's leaves with extractPointCloud(). getLabels() returns
    # -1 (unknown / never observed), 0 (free), or 1 (occupied) for arbitrary
    # points -- the typical collision-query use. Swap the block above for:
    #
    # center = (octree.getMetricMin() + octree.getMetricMax()) / 2
    # dimension = np.array([64, 64, 64])
    # origin = center - dimension / 2 * resolution
    # aabb_min = origin - resolution / 2
    # aabb_max = origin + dimension * resolution + resolution / 2
    # grid = np.full(dimension, -1, np.int32)
    # transform = tf.scale_and_translate(scale=resolution, translate=origin)
    # points = trimesh.voxel.VoxelGrid(
    #     encoding=grid, transform=transform
    # ).points
    # labels = octree.getLabels(points)
    # occupied = points[labels == 1]
    # empty = points[labels == 0]

    visualize(
        occupied=occupied,
        empty=empty,
        K=K,
        width=camera_info["width"],
        height=camera_info["height"],
        rgb=rgb,
        pcd=pcd,
        mask=mask,
        resolution=resolution,
        aabb=(aabb_min, aabb_max),
    )


if __name__ == "__main__":
    main()
