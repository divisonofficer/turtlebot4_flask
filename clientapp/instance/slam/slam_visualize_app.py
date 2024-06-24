import time
import cv2
import numpy as np
import sys

sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")
sys.path.append("./instance/slam/depthai_slam")
from depthai_slam.slam import process
from depthai_slam.display import Display
from depthai_slam.pointmap import PointMap
import open3d
from open3d.visualization.rendering import OffscreenRenderer, MaterialRecord
from open3d.geometry import PointCloud
from open3d.utility import Vector3dVector
from flask import Flask, request, jsonify
import argparse


class SlamVisualizer:
    def __init__(self):
        self.width = 1280
        self.height = 720
        self.pcd = PointCloud()
        self.pmap = PointMap()

    def prepare_image(self, image: np.ndarray):
        if image is None:
            return
        if len(image.shape) == 2:
            image = np.expand_dims(image, axis=2)
        img, tripoints3d, kpts, matches = process(image)
        xyz = self.pmap.collect_points(tripoints3d)
        if not xyz is None:
            self.xyz = xyz
            self.img = img
            self.kpts = kpts
            self.matches = matches
            self.pcd.points = Vector3dVector(xyz)
            print(self.pcd)
            self.visualize()

    def visualize(self):
        pass


class SlamVisualizerRender(SlamVisualizer):
    def __init__(self):
        super().__init__()
        self.renderer = OffscreenRenderer(self.width, self.height)
        self.renderer.scene.set_background([1, 1, 1, 1])

    def visualize(self):
        print("Rendering Geometry Internally")
        self.renderer.scene.remove_geometry("point_cloud")
        material = MaterialRecord()
        material.base_color = [1.0, 0.0, 0.0, 1.0]
        self.renderer.scene.add_geometry("point_cloud", self.pcd, material)
        output = self.renderer.render_to_image()
        print(output)
        output = np.asarray(output)
        print(output.shape)
        cv2.imwrite("depth.png", output)


import os
from pynput import keyboard


class SlamVisualizerX11(SlamVisualizer):
    def __init__(self):
        super().__init__()
        self.viz = open3d.visualization.Visualizer()
        self.viz.create_window(width=self.width, height=self.height)
        self.display = Display()

    def visualize(self):
        print("Rendering Geometry on X11")
        self.viz.remove_geometry(self.pcd)
        self.viz.add_geometry(self.pcd)
        self.viz.update_geometry(self.pcd)
        self.viz.poll_events()
        self.viz.update_renderer()

    def move_camera(self, key: keyboard.Key):
        viz_control = self.viz.get_view_control()
        if key == keyboard.Key.up:
            viz_control.rotate(0.0, 0.1)
        elif key == keyboard.Key.down:
            viz_control.rotate(0.0, -0.1)
        elif key == keyboard.Key.left:
            viz_control.rotate(0.1, 0.0)
        elif key == keyboard.Key.right:
            viz_control.rotate(-0.1, 0.0)
        elif key == keyboard.KeyCode(char="w"):
            viz_control.translate(0.0, 0.0, 0.1)
        elif key == keyboard.KeyCode(char="s"):
            viz_control.translate(0.0, 0.0, -0.1)
        elif key == keyboard.KeyCode(char="a"):
            viz_control.translate(-0.1, 0.0, 0.0)
        elif key == keyboard.KeyCode(char="d"):
            viz_control.translate(0.1, 0.0, 0.0)
        elif key == keyboard.Key.space:
            viz_control.translate(0.0, 0.0, 0.1)
        elif key == keyboard.Key.shift:
            viz_control.translate(0.0, 0.0, -0.1)
        self.viz.poll_events()
        self.viz.update_renderer()
        print("Camera Moved: ", key)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image_path", help="Path to the image file")
    parser.add_argument("-W", "--Window", help="Window type", default="X11")
    args = parser.parse_args()
    image_path = args.image_path
    window = args.Window
    visualizer: SlamVisualizer
    if window == "X11":
        visualizer = SlamVisualizerX11()
    else:
        visualizer = SlamVisualizerRender()
    print(window)
    if isinstance(visualizer, SlamVisualizerX11):
        keyboard_input = keyboard.Listener(on_press=visualizer.move_camera)
    while True:

        if os.path.exists(image_path):
            image = cv2.imread(image_path)
            visualizer.prepare_image(image)
            os.remove(image_path)
        time.sleep(1)
