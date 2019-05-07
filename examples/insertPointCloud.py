#!/usr/bin/env python

import glooey
import imgviz
import numpy as np
import pyglet
import trimesh
import trimesh.transformations as tf
import trimesh.viewer

import octomap


def pointcloud_from_depth(depth, fx, fy, cx, cy):
    assert depth.dtype.kind == 'f', 'depth must be float and have meter values'

    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    pc = np.dstack((x, y, z))

    return pc


def opengl_camera_transform(transform=None):
    if transform is None:
        transform = np.eye(4)
    return transform @ tf.rotation_matrix(np.deg2rad(-180), [1, 0, 0])


def labeled_scene_widget(scene, label):
    vbox = glooey.VBox()
    vbox.add(glooey.Label(text=label, color=(255, 255, 255)), size=0)
    vbox.add(trimesh.viewer.SceneWidget(scene))
    return vbox


def main():
    data = imgviz.data.arc2017()
    camera_info = data['camera_info'][()]
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
        maxrange=2,
    )
    root = octree.begin_tree()
    occupied = []
    empty = []
    for node in root:
        if node.isLeaf():
            coord = node.getCoordinate()
            if octree.isNodeOccupied(node):
                occupied.append(coord)
            else:
                empty.append(coord)
    occupied = np.array(occupied)
    empty = np.array(empty)

    # -------------------------------------------------------------------------

    window = pyglet.window.Window(
        width=int(640 * 0.9 * 3), height=int(480 * 0.9)
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
        resolution=(camera_info['width'], camera_info['height']),
        focal=(K[0, 0], K[1, 1]),
        transform=np.eye(4),
    )
    camera_marker = trimesh.creation.camera_marker(camera, marker_height=0.1)

    # initial camera pose
    camera.transform = np.array(
        [[0.73256052, -0.28776419,  0.6168848,   0.66972396],
         [-0.26470017, -0.95534823, -0.13131483, -0.12390466],
         [0.62712751, -0.06709345, -0.77602162, -0.28781298],
         [0.,          0.,          0.,          1., ]],
    )

    geom = trimesh.PointCloud(vertices=pcd[mask], colors=rgb[mask])
    scene = trimesh.Scene(camera=camera, geometry=[geom, camera_marker])
    hbox.add(labeled_scene_widget(scene, label='pointcloud'))

    geom = trimesh.voxel.multibox(
        occupied, pitch=resolution, colors=[1., 0, 0, 0.5]
    )
    scene = trimesh.Scene(camera=camera, geometry=[geom, camera_marker])
    hbox.add(labeled_scene_widget(scene, label='occupied'))

    geom = trimesh.voxel.multibox(
        empty, pitch=resolution, colors=[0.5, 0.5, 0.5, 0.5]
    )
    scene = trimesh.Scene(camera=camera, geometry=[geom, camera_marker])
    hbox.add(labeled_scene_widget(scene, label='empty'))

    gui.add(hbox)
    pyglet.app.run()


if __name__ == '__main__':
    main()
