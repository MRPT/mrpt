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

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <mrpt/vision/pnp/epnp.h>
#include <mrpt/vision/pnp/upnp.h>
#include <mrpt/vision/pnp/dls.h>

namespace pnp
{
	class CPnP
	{
		public:
			template<typename Derived>
			int CPnP_dls(Eigen::MatrixBase<Derived>& obj_pts, Eigen::MatrixBase<Derived>& img_pts, int n, Eigen::MatrixBase<Derived>& cam_intrinsic, Eigen::MatrixBase<Derived>& pose_mat);

			template<typename Derived>
			int CPnP_epnp(Eigen::MatrixBase<Derived>& obj_pts, Eigen::MatrixBase<Derived>& img_pts, int n, Eigen::MatrixBase<Derived>& cam_intrinsic, Eigen::MatrixBase<Derived>& pose_mat);

			template<typename Derived>
			int CPnP_upnp(Eigen::MatrixBase<Derived>& obj_pts, Eigen::MatrixBase<Derived>& img_pts, int n, Eigen::MatrixBase<Derived>& cam_intrinsic, Eigen::MatrixBase<Derived>& pose_mat);
	}pnp_algos;
}

#endif
