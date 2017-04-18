#!/usr/bin/env python
from PnPAlgos import PnPAlgos

import numpy as np

import random

# Define number of points and camera parameters
n=6
f=1.0
cx=0.0
cy=0.0

# Define object points and image points
obj_pts=np.array([[0,-15,-3],[0,0,-50.0],[2,0,35],[5,-40,25],[10,15,9],[-20,50,7]])
img_pts=np.empty([n,2])

# Define camera extrinsic matrix
R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
t=np.array([0.0,0.0,100.0])

# Compute image points based on actual extrinsic matrix and add noise to measurements
for i in range(0,6):
    pt=np.dot(R,obj_pts[i,:])+t
    img_pts[i,:]= np.array([pt[0]/pt[2] +random.uniform(-0.01,0.01), pt[1]/pt[2]+random.uniform(-0.01,0.01)])
    
pose_mat=np.empty([4,4])
pose_mat_orig=np.empty([4,4])

pose_mat_orig[3,:]=np.array([0,0,0,1])
pose_mat_orig[0:3,3]=t;
pose_mat_orig[0:3,0:3]=R
cam_intrinsic=np.array([[f,0.0,cx],[0.0,f,cy],[0.0, 0.0, 1.0]])

# Use the c-library to compute the pose 
pnp_bundle = PnPAlgos(n)
pnp_bundle.pnpalgo1(obj_pts,img_pts, 6, cam_intrinsic, pose_mat)
pose_mat=np.transpose(pose_mat)

# Display the results
print "obj_pts=\n", obj_pts
print "img_pts=\n",img_pts
print "pose_mat_orig=\n", pose_mat_orig
print "pose_mat_est=\n",pose_mat
print "cam_mat=\n",cam_intrinsic