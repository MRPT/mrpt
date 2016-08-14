/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <mrpt/config.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/pnp_algos.h>
using namespace mrpt::vision::pnp;

#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
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

#if MRPT_HAS_OPENCV

    int CPnP::dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv(3,3,CV_32F), t_cv(3,1,CV_32F);
       
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        mrpt::vision::dls d(obj_pts_cv, img_pts_cv);
        int ret = d.compute_pose(R_cv,t_cv);
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        return ret;
    }

    int CPnP::epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        mrpt::vision::epnp e(cam_in_cv, obj_pts_cv, img_pts_cv);
        e.compute_pose(R_cv,t_cv);
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        return 1;
    }

    int CPnP::upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose().block(0,0,n,2), obj_pts_eig=obj_pts.transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        mrpt::vision::upnp u(cam_in_cv, obj_pts_cv, img_pts_cv);
        u.compute_pose(R_cv,t_cv);
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        return 1;
    }
#endif

int CPnP::p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::p3p p(cam_in_eig);
    int ret = p.solve(R_eig,t_eig, obj_pts_eig, img_pts_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}


int CPnP::rpnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::rpnp r(obj_pts_eig, img_pts_eig, cam_in_eig, n);
    int ret = r.compute_pose(R_eig,t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}

int CPnP::ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R(3,3);
	Eigen::VectorXd t(3);
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();

	mrpt::vision::ppnp p(obj_pts_,img_pts_, cam_intrinsic);
	
	int ret = p.compute_pose(R,t,n);
	
	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}

int CPnP::posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();

	mrpt::vision::POSIT p(obj_pts_,img_pts_, cam_intrinsic, n);
	
	int ret = p.compute_pose(R,t);
    
	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}

int CPnP::lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose();
    
    mrpt::vision::lhm l(obj_pts_, img_pts_, cam_intrinsic, n);
    
    int ret = l.compute_pose(R,t);
    
    Eigen::Quaterniond q(R);
    
    pose_mat<<t,q.vec();
    
    return ret;
    
}

int CPnP::so3(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.transpose(), img_pts_=img_pts.transpose(), cam_intrinsic_ = cam_intrinsic.transpose();
    
    mrpt::vision::p3p p(cam_intrinsic_);
    p.solve(R,t, obj_pts_, img_pts_);
    
    mrpt::vision::so3 s(obj_pts_, img_pts_, cam_intrinsic_, n);
    int ret = s.compute_pose(R,t);
    
    Eigen::Quaterniond q(R);
    
    pose_mat<<t,q.vec();
    
    return ret;
    
}
