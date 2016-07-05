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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


namespace pnp
{
	class CPnP
	{
        public:
			int CPnP_ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
	
			int CPnP_posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
	
            int CPnP_lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
	
            #if MRPT_HAS_OPENCV
                int CPnP_dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

                int CPnP_epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

                int CPnP_upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
                
                int CPnP_p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
			#endif
			};
	
}

#endif
