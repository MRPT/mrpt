/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/pnp_algos.h>

#include <iostream>

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Core>
#include <Eigen/Dense>

#include <mrpt/otherlibs/do_opencv_includes.h>
#include <opencv2/core/eigen.hpp>

#include "dls.h"
#include "epnp.h"
#include "upnp.h"
#include "p3p.h"
#include "ppnp.h"
#include "posit.h"
#include "lhm.h"
#include "rpnp.h"
#include "so3.h"

bool mrpt::vision::pnp::CPnP::dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
#if MRPT_HAS_OPENCV

    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
    Eigen::Matrix3d R_eig; 
    cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv(3,3,CV_32F), t_cv(3,1,CV_32F);
   
    cv::eigen2cv(cam_in_eig, cam_in_cv);
    cv::eigen2cv(img_pts_eig, img_pts_cv);
    cv::eigen2cv(obj_pts_eig, obj_pts_cv);
    
    mrpt::vision::pnp::dls d(obj_pts_cv, img_pts_cv);
    bool ret = d.compute_pose(R_cv,t_cv);
    
    cv::cv2eigen(R_cv, R_eig);
    cv::cv2eigen(t_cv, t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
#else
	THROW_EXCEPTION("This function requires compiling MRPT against OpenCV!");
#endif
}

bool mrpt::vision::pnp::CPnP::epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
#if MRPT_HAS_OPENCV
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
    Eigen::Matrix3d R_eig; 
    cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
    
    cv::eigen2cv(cam_in_eig, cam_in_cv);
    cv::eigen2cv(img_pts_eig, img_pts_cv);
    cv::eigen2cv(obj_pts_eig, obj_pts_cv);
    
    mrpt::vision::pnp::epnp e(cam_in_cv, obj_pts_cv, img_pts_cv);
    e.compute_pose(R_cv,t_cv);
    
    cv::cv2eigen(R_cv, R_eig);
    cv::cv2eigen(t_cv, t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return true;
#else
	THROW_EXCEPTION("This function requires compiling MRPT against OpenCV!");
#endif
}

bool mrpt::vision::pnp::CPnP::upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
#if MRPT_HAS_OPENCV
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
    Eigen::Matrix3d R_eig; 
    cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
    
    cv::eigen2cv(cam_in_eig, cam_in_cv);
    cv::eigen2cv(img_pts_eig, img_pts_cv);
    cv::eigen2cv(obj_pts_eig, obj_pts_cv);
    
    mrpt::vision::pnp::upnp u(cam_in_cv, obj_pts_cv, img_pts_cv);
    u.compute_pose(R_cv,t_cv);
    
    cv::cv2eigen(R_cv, R_eig);
    cv::cv2eigen(t_cv, t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return true;
#else
	THROW_EXCEPTION("This function requires compiling MRPT against OpenCV!");
#endif
}


bool mrpt::vision::pnp::CPnP::p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::pnp::p3p p(cam_in_eig);
    bool ret = p.solve(R_eig,t_eig, obj_pts_eig, img_pts_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}


bool mrpt::vision::pnp::CPnP::rpnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::pnp::rpnp r(obj_pts_eig, img_pts_eig, cam_in_eig, n);
    bool ret = r.compute_pose(R_eig,t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}

bool mrpt::vision::pnp::CPnP::ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R(3,3);
	Eigen::VectorXd t(3);
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();

	mrpt::vision::pnp::ppnp p(obj_pts_,img_pts_, cam_intrinsic);
	
	bool ret = p.compute_pose(R,t,n);
	
	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}

bool mrpt::vision::pnp::CPnP::posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();

	mrpt::vision::pnp::posit p(obj_pts_,img_pts_, cam_intrinsic, n);
	
	bool ret = p.compute_pose(R,t);

	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}

bool mrpt::vision::pnp::CPnP::lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();
    
    mrpt::vision::pnp::lhm l(obj_pts_, img_pts_, cam_intrinsic, n);
    
    bool ret = l.compute_pose(R,t);
    
    Eigen::Quaterniond q(R);
    
    pose_mat<<t,q.vec();
    
    return ret;
    
}

bool mrpt::vision::pnp::CPnP::so3(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose(), cam_intrinsic_ = cam_intrinsic.transpose();
    
    mrpt::vision::pnp::p3p p(cam_intrinsic_);
    p.solve(R,t, obj_pts_, img_pts_);
    
    mrpt::vision::pnp::so3 s(obj_pts_, img_pts_, cam_intrinsic_, n);
    bool ret = s.compute_pose(R,t);
    
    Eigen::Quaterniond q(R);
    
    pose_mat<<t,q.vec();
    
    return ret;
    
}
