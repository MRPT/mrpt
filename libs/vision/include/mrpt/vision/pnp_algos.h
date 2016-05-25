/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef __pnp_algos_h
#define __pnp_algos_h

#include <iostream>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;

#include <mrpt/vision/epnp.h>
#include <mrpt/vision/upnp.h>
#include <mrpt/vision/dls.h>

template<typename Derived>
int pnpalgo_dls(MatrixBase<Derived>& obj_pts, MatrixBase<Derived>& img_pts, int n, MatrixBase<Derived>& cam_intrinsic, MatrixBase<Derived>& pose_mat);

template<typename Derived>
int pnpalgo_epnp(MatrixBase<Derived>& obj_pts, MatrixBase<Derived>& img_pts, int n, MatrixBase<Derived>& cam_intrinsic, MatrixBase<Derived>& pose_mat);

template<typename Derived>
int pnpalgo_upnp(MatrixBase<Derived>& obj_pts, MatrixBase<Derived>& img_pts, int n, MatrixBase<Derived>& cam_intrinsic, MatrixBase<Derived>& pose_mat);

#endif
