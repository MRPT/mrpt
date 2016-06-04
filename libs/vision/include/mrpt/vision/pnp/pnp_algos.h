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

namespace pnp
{
	class CPnP
	{
		public:
			int CPnP_dls(Eigen::MatrixXd& obj_pts, Eigen::MatrixXd& img_pts, int n, Eigen::MatrixXd& cam_intrinsic, Eigen::MatrixXd& pose_mat);

			int CPnP_epnp(Eigen::MatrixXd& obj_pts, Eigen::MatrixXd& img_pts, int n, Eigen::MatrixXd& cam_intrinsic, Eigen::MatrixXd& pose_mat);

			int CPnP_upnp(Eigen::MatrixXd& obj_pts, Eigen::MatrixXd& img_pts, int n, Eigen::MatrixXd& cam_intrinsic, Eigen::MatrixXd& pose_mat);
	};
	
}

#endif
