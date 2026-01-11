#!/bin/env python3

import mrpt.viz
import mrpt.poses
import numpy as np

# 1. Create a scene
scene = mrpt.viz.Scene()

# 2. Add a 3D model via Assimp
model = mrpt.viz.CAssimpModel()
loadOk = model.loadScene("robot_base.obj")
print(f"Loaded scene file ok: {loadOk}")
scene << model

# 3. Add a Point Cloud using NumPy
pts = np.random.rand(1000, 3).astype(np.float32)
# pc = mrpt.viz.create_point_cloud(pts, color=(0, 255, 0))
pc.pose = mrpt.poses.CPose3D(1.0, 0, 0, 0, 0, 0)  # Move it
scene << pc

# 4. Manipulate the camera
cam = scene.getViewport().getCamera()
cam.setAzimuthDegrees(45)
cam.setZoomDistance(15)

print(f"Scene objects in main viewport: {len(scene.getViewport())}")
