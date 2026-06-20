#!/usr/bin/env python

import glooey
import imgviz
import numpy as np
import pyglet
import trimesh
import trimesh.transformations as tf
import trimesh.viewer
from insertPointCloud import labeled_scene_widget
from insertPointCloud import pointcloud_from_depth

import octomap


def build_color_octree(pcd, rgb, mask, resolution, maxrange):
    octree = octomap.ColorOcTree(resolution)
    octree.insertPointCloud(
        pointcloud=pcd[mask],
        origin=np.array([0, 0, 0], dtype=float),
        maxrange=maxrange,
    )
    for point, color in zip(pcd[mask], rgb[mask]):
        octree.averageNodeColor(
            point, int(color[0]), int(color[1]), int(color[2])
        )
    octree.updateInnerOccupancy()
    return octree


def extract_colored_voxels(octree, resolution):
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
        indices = np.column_stack(
            np.nonzero(np.ones((dimension, dimension, dimension)))
        )
        points.append(origin + indices * resolution)
        colors.append(np.tile(it.getColor(), (len(indices), 1)))

    if not points:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    return (
        np.concatenate(points, axis=0),
        np.concatenate(colors, axis=0).astype(np.uint8),
    )


def visualize(
    occupied,
    occupied_colors,
    K,
    width,
    height,
    rgb,
    pcd,
    mask,
    resolution,
    aabb,
):
    window = pyglet.window.Window(
        width=int(640 * 0.9 * 2), height=int(480 * 0.9)
    )

    @window.event
    def on_key_press(symbol, modifiers):
        if modifiers == 0:
            if symbol == pyglet.window.key.Q:
                window.on_close()

    gui = glooey.Gui(window)
    hbox = glooey.HBox()
    hbox.set_padding(5)

    camera = trimesh.scene.Camera(
        resolution=(width, height), focal=(K[0, 0], K[1, 1])
    )
    camera_marker = trimesh.creation.camera_marker(camera, marker_height=0.1)
    opencv_from_opengl = tf.rotation_matrix(np.pi, [1, 0, 0])
    for geometry in camera_marker:
        geometry.apply_transform(opencv_from_opengl)

    camera_transform = np.array(
        [
            [0.73256052, -0.28776419, 0.6168848, 0.66972396],
            [-0.26470017, -0.95534823, -0.13131483, -0.12390466],
            [0.62712751, -0.06709345, -0.77602162, -0.28781298],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )

    aabb_min, aabb_max = aabb
    bbox = trimesh.path.creation.box_outline(
        aabb_max - aabb_min,
        tf.translation_matrix((aabb_min + aabb_max) / 2),
    )

    geom = trimesh.PointCloud(vertices=pcd[mask], colors=rgb[mask])
    scene = trimesh.Scene(camera=camera, geometry=[bbox, geom, camera_marker])
    scene.camera_transform = camera_transform
    hbox.add(labeled_scene_widget(scene, label="pointcloud"))

    colors = np.column_stack(
        (occupied_colors, np.full(len(occupied_colors), 255, dtype=np.uint8))
    )
    geom = trimesh.voxel.ops.multibox(
        occupied, pitch=resolution, colors=colors
    )
    scene = trimesh.Scene(camera=camera, geometry=[bbox, geom, camera_marker])
    scene.camera_transform = camera_transform
    hbox.add(labeled_scene_widget(scene, label="occupied (color)"))

    gui.add(hbox)
    pyglet.app.run()


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

    resolution = 0.01
    octree = build_color_octree(
        pcd=pcd, rgb=rgb, mask=mask, resolution=resolution, maxrange=2
    )
    occupied, occupied_colors = extract_colored_voxels(octree, resolution)

    visualize(
        occupied=occupied,
        occupied_colors=occupied_colors,
        K=K,
        width=camera_info["width"],
        height=camera_info["height"],
        rgb=rgb,
        pcd=pcd,
        mask=mask,
        resolution=resolution,
        aabb=(octree.getMetricMin(), octree.getMetricMax()),
    )


if __name__ == "__main__":
    main()
