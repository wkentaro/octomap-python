#!/usr/bin/env python

"""A 2x2 trimesh viewer with camera-synced panes, used by the examples."""

import glooey
import numpy as np
import pyglet
import trimesh
import trimesh.transformations as tf
import trimesh.viewer


def pointcloud_from_depth(depth, fx, fy, cx, cy):
    assert depth.dtype.kind == "f", "depth must be float and have meter values"

    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    return np.dstack((x, y, z))


class SyncedSceneWidget(trimesh.viewer.SceneWidget):
    """A SceneWidget whose camera is kept in sync with its peers.

    All synced widgets share a single trackball, so a press/drag/scroll on
    any one of them moves every camera together. After each interaction the
    peers' scenes are pointed at the shared pose and redrawn.
    """

    def __init__(self, scene, **kwargs):
        super().__init__(scene, **kwargs)
        self.peers = []

    def sync(self, peers):
        ball = peers[0].view["ball"]
        self.view["ball"] = ball
        self.scene.camera_transform = ball.pose
        self.peers = [peer for peer in peers if peer is not self]

    def _sync_peers(self):
        pose = self.view["ball"].pose
        for peer in self.peers:
            peer.scene.camera_transform = pose
            peer._draw()

    def on_mouse_press(self, x, y, buttons, modifiers):
        super().on_mouse_press(x, y, buttons, modifiers)
        self._sync_peers()

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        super().on_mouse_drag(x, y, dx, dy, buttons, modifiers)
        self._sync_peers()

    def on_mouse_scroll(self, x, y, dx, dy):
        super().on_mouse_scroll(x, y, dx, dy)
        self._sync_peers()


def _labeled_scene_widget(scene, label):
    widget = SyncedSceneWidget(scene, background=(1.0, 1.0, 1.0, 1.0))

    vbox = glooey.VBox()
    vbox.add(glooey.Label(text=label, color=(0, 0, 0)), size=0)
    vbox.add(widget)
    return vbox, widget


def visualize(
    *,
    pointcloud,
    occupied,
    occupied_color,
    empty,
    K,
    width,
    height,
    resolution,
    aabb,
):
    window = pyglet.window.Window(
        width=int(640 * 0.9 * 2), height=int(480 * 0.9 * 2)
    )

    @window.event
    def on_key_press(symbol, modifiers):
        if modifiers == 0:
            if symbol == pyglet.window.key.Q:
                window.on_close()

    gui = glooey.Gui(window)
    grid = glooey.Grid(num_rows=2, num_cols=2)
    grid.set_padding(5)

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

    # initial camera pose
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

    widgets = []

    def add_panel(row, col, label, geom):
        scene = trimesh.Scene(
            camera=camera, geometry=[bbox, geom, camera_marker]
        )
        scene.camera_transform = camera_transform
        vbox, widget = _labeled_scene_widget(scene, label)
        widgets.append(widget)
        grid.add(row, col, vbox)

    pcd_points, pcd_colors = pointcloud
    add_panel(
        0,
        0,
        "pointcloud",
        trimesh.PointCloud(vertices=pcd_points, colors=pcd_colors),
    )

    add_panel(
        0,
        1,
        "occupied",
        trimesh.voxel.ops.multibox(
            occupied, pitch=resolution, colors=[1.0, 0, 0, 0.5]
        ),
    )

    voxel_points, voxel_colors = occupied_color
    voxel_colors = np.column_stack(
        (voxel_colors, np.full(len(voxel_colors), 255, dtype=np.uint8))
    )
    add_panel(
        1,
        0,
        "occupied (color)",
        trimesh.voxel.ops.multibox(
            voxel_points, pitch=resolution, colors=voxel_colors
        ),
    )

    add_panel(
        1,
        1,
        "empty",
        trimesh.voxel.ops.multibox(
            empty, pitch=resolution, colors=[0.5, 0.5, 0.5, 0.5]
        ),
    )

    for widget in widgets:
        widget.sync(widgets)

    gui.add(grid)
    pyglet.app.run()
