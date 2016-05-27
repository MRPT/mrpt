#!/usr/bin/env python
import sys
sys.path.append("../../build/lib")
import numpy as np
import matplotlib.pyplot as plt
import random

import pymrpt

def vector2RotMat(vec, theta=0):
    """Rodrigues rotation formula"""
    n_check=np.linalg.norm(vec)
    
    kx=vec[0]/n_check
    ky=vec[1]/n_check
    kz=vec[2]/n_check
    
    K=np.matrix([[0, -kz, ky],[kz, 0, -kx],[-ky, kx, 0]])
    I=np.identity(3)
    
    R=I+K*np.sin(theta)+K*K*(1-np.cos(theta))
    R=np.array(R)
    
    return R

def display_comparison_plot(t, arr, names, title, xtitle, ytitle):
    for i in np.arange(0,len(arr)):
        plt.plot(t,arr[i,:],label=names[i])
        
    plt.xlabel(xtitle)
    plt.ylabel(ytitle)
    plt.title(title)
    ax=plt.gca()
    ax.set_ylim([0,10])

# Define number of points and camera parameters
n=6
f=1.0
cx=0.0
cy=0.0

# Instantiate pnp module
pnp = pymrpt.pnp(n)

# Define object points and image points
obj_pts=np.array([[0,-15,-3],[0,0,-50.0],[2,0,35],[5,-40,25],[10,15,9],[-20,50,7]])
img_pts=np.empty([n,2])
pose_epnp=np.empty([6,1])
pose_upnp=np.empty([6,1])
pose_dls=np.empty([6,1])
pose_mat_orig=np.empty([4,4])

n_iter=100
n_algos=3

err_t_epnp=[]
err_t_dls=[]
err_t_upnp=[]

for it in np.arange(0,n_iter):

    # Define camera extrinsic matrix
    v=2*np.random.random([3]) - np.array([1,1,1])
    v=v/np.linalg.norm(v)
    
    #R=np.array([[0.841986, -0.352662, -0.408276],[0.308904,  0.935579, -0.171085],[0.442309, 0.0179335,  0.896683]])
    R = vector2RotMat(v, np.pi*2/3)
    t=np.array([0.0,0.0,100.0])
    
    # Compute image points based on actual extrinsic matrix and add noise to measurements
    for i in range(0,6):
        pt=np.dot(R,obj_pts[i,:])+t
        img_pts[i,:]= np.array([pt[0]/pt[2] +random.uniform(-0.01,0.01), pt[1]/pt[2]+random.uniform(-0.01,0.01)])
        
    
    pose_mat_orig[3,:]=np.array([0,0,0,1])
    pose_mat_orig[0:3,3]=t;
    pose_mat_orig[0:3,0:3]=R
    cam_intrinsic=np.array([[f,0.0,cx],[0.0,f,cy],[0.0, 0.0, 1.0]])
    
    # Use the c-library to compute the pose 
    pnp.epnp_solve(obj_pts,img_pts, 6, cam_intrinsic, pose_epnp)
    pnp.dls_solve(obj_pts, img_pts, 6, cam_intrinsic, pose_dls)
    pnp.upnp_solve(obj_pts,img_pts, 6, cam_intrinsic, pose_upnp)

    t_epnp=np.concatenate(pose_epnp[0:3])
    t_dls=np.concatenate(pose_dls[0:3])
    t_upnp=np.concatenate(pose_upnp[0:3])
    
    err_t_epnp.append(np.linalg.norm(t-t_epnp))
    err_t_dls.append(np.linalg.norm(t-t_dls))
    err_t_upnp.append(np.linalg.norm(t-t_upnp))

err_algos=np.array(err_t_epnp + err_t_dls + err_t_upnp)
err_algos=err_algos.reshape(n_algos,n_iter)

it=np.arange(0,n_iter)

plt.figure(1)
display_comparison_plot(it, err_algos, names=['epnp','dls','upnp'], title='Translation Error Plot', xtitle='Iteration', ytitle='e')
plt.legend()
plt.show()

# Display the results
"""
print "obj_pts=\n", obj_pts
print "img_pts=\n",img_pts
print "pose_mat_orig=\n", pose_mat_orig
print "pose_mat_est=\n",pose_epnp
print "pose_mat_est1=\n",pose_dls
print "cam_mat=\n",cam_intrinsic
"""