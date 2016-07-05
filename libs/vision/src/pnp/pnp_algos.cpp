/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision/pnp_algos.h>
using namespace pnp;

#include <iostream>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "dls.h"
#include "epnp.h"
#include "upnp.h"
#include "p3p.h"
#include "ppnp.h"
#include "posit.h"
#include "lhm.h"

#if MRPT_HAS_OPENCV
    int CPnP::CPnP_dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.array().transpose(), img_pts_eig=img_pts.array().transpose(), obj_pts_eig=obj_pts.array().transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv(3,3,CV_32F), t_cv(3,1,CV_32F);
        
        //cout<<"cam_in="<<endl<<cam_in_eig<<endl<<endl;
        //cout<<"obj_pts="<<endl<<obj_pts_eig<<endl<<endl;
        //cout<<"img_pts="<<endl<<img_pts_eig<<endl<<endl;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        //cout<<cam_in_cv<<endl;
        //cout<<img_pts_cv<<endl;
        //cout<<obj_pts_cv<<endl;
        
        mrpt::vision::dls d(obj_pts_cv, img_pts_cv);
        int ret = d.compute_pose(R_cv,t_cv);
        
        //cout<<R_cv<<endl;
        //cout<<t_cv<<endl;
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        //cout<<"R_eig="<<endl<<R_eig<<endl<<endl;
        //cout<<"t_eig="<<endl<<t_eig<<endl<<endl;
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        //pose_mat.block(0,0,3,3)=R_eig;
        //pose_mat.block(0,3,3,1)=t_eig;
        //pose_mat.row(3)<<0,0,0,1;
        //cout<<"t_eig="<<endl<<t_eig<<endl<<endl;
        //cout<<"q_eig="<<endl<<q.vec()<<endl<<endl;
        //cout<<"pose_eig="<<endl<<pose_mat<<endl<<endl;
        //cout<<"pose_cv="<<endl<<R_cv<<endl<<endl;
        
        return ret;
    }

    int CPnP::CPnP_epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.array().transpose(), img_pts_eig=img_pts.array().transpose(), obj_pts_eig=obj_pts.array().transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
        
        //cout<<"cam_in="<<endl<<cam_in_eig<<endl<<endl;
        //cout<<"obj_pts="<<endl<<obj_pts_eig<<endl<<endl;
        //cout<<"img_pts="<<endl<<img_pts_eig<<endl<<endl;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        //cout<<cam_in_cv<<endl;
        //cout<<img_pts_cv<<endl;
        //cout<<obj_pts_cv<<endl;
        
        mrpt::vision::epnp e(cam_in_cv, obj_pts_cv, img_pts_cv);
        e.compute_pose(R_cv,t_cv);
        
        //cout<<R_cv<<endl;
        //cout<<t_cv<<endl;
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        //cout<<"R_eig="<<endl<<R_eig<<endl<<endl;
        //cout<<"t_eig="<<endl<<t_eig<<endl<<endl;
        
        //pose_mat.block(0,0,3,3)=R_eig;
        //pose_mat.block(0,3,3,1)=t_eig;
        //pose_mat.row(3)<<0,0,0,1;
        
        //cout<<"pose_mat="<<endl<<pose_mat_eig<<endl<<endl;
        
        return 1;
    }

    int CPnP::CPnP_upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.array().transpose(), img_pts_eig=img_pts.array().transpose(), obj_pts_eig=obj_pts.array().transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,n,CV_32F), obj_pts_cv(3,n,CV_32F), R_cv, t_cv;
        
        //cout<<"cam_in="<<endl<<cam_in_eig<<endl<<endl;
        //cout<<"obj_pts="<<endl<<obj_pts_eig<<endl<<endl;
        //cout<<"img_pts="<<endl<<img_pts_eig<<endl<<endl;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        //cout<<cam_in_cv<<endl;
        //cout<<img_pts_cv<<endl;
        //cout<<obj_pts_cv<<endl;
        
        mrpt::vision::upnp u(cam_in_cv, obj_pts_cv, img_pts_cv);
        u.compute_pose(R_cv,t_cv);
        
        //cout<<R_cv<<endl;
        //cout<<t_cv<<endl;
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        //cout<<"R_eig="<<endl<<R_eig<<endl<<endl;
        //cout<<"t_eig="<<endl<<t_eig<<endl<<endl;
        
        //pose_mat.block(0,0,3,3)=R_eig;
        //pose_mat.block(0,3,3,1)=t_eig;
        //pose_mat.row(3)<<0,0,0,1;
        
        //cout<<"pose_mat="<<endl<<pose_mat_eig<<endl<<endl;
        
        return 1;
    }

    int CPnP::CPnP_p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
        Eigen::MatrixXd cam_in_eig=cam_intrinsic.array().transpose(), img_pts_eig=img_pts.array().transpose(), obj_pts_eig=obj_pts.array().transpose(), t_eig;
        Eigen::Matrix3d R_eig; 
        cv::Mat cam_in_cv(3,3,CV_32F), img_pts_cv(2,3,CV_32F), obj_pts_cv(3,3,CV_32F), R_cv, t_cv;
        
        //cout<<"cam_in="<<endl<<cam_in_eig<<endl<<endl;
        //cout<<"obj_pts="<<endl<<obj_pts_eig<<endl<<endl;
        //cout<<"img_pts="<<endl<<img_pts_eig<<endl<<endl;
        
        cv::eigen2cv(cam_in_eig, cam_in_cv);
        cv::eigen2cv(img_pts_eig, img_pts_cv);
        cv::eigen2cv(obj_pts_eig, obj_pts_cv);
        
        //cout<<cam_in_cv<<endl;
        //cout<<img_pts_cv<<endl;
        //cout<<obj_pts_cv<<endl;
        
        mrpt::vision::p3p p(cam_in_cv);
        int ret = p.solve(R_cv,t_cv, obj_pts_cv, img_pts_cv);
        
        //cout<<R_cv<<endl;
        //cout<<t_cv<<endl;
        
        cv::cv2eigen(R_cv, R_eig);
        cv::cv2eigen(t_cv, t_eig);
        
        Eigen::Quaterniond q(R_eig);
        
        pose_mat << t_eig,q.vec();
        
        //cout<<"R_eig="<<endl<<R_eig<<endl<<endl;
        //cout<<"t_eig="<<endl<<t_eig<<endl<<endl;
        
        //pose_mat.block(0,0,3,3)=R_eig;
        //pose_mat.block(0,3,3,1)=t_eig;
        //pose_mat.row(3)<<0,0,0,1;
        
        //cout<<"pose_mat="<<endl<<pose_mat_eig<<endl<<endl;
        
        return ret;
    }
#endif


int CPnP::CPnP_ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R(3,3);
	Eigen::VectorXd t(3);
	
	Eigen::MatrixXd obj_pts_=obj_pts.array().transpose(), img_pts_=img_pts.array().transpose();

	mrpt::vision::ppnp p(obj_pts_,img_pts_, cam_intrinsic);
	
	int ret = p.compute_pose(R,t,n);
	
	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}

int CPnP::CPnP_posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{	
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.array().transpose(), img_pts_=img_pts.array().transpose();

	mrpt::vision::POSIT p(obj_pts_,img_pts_, cam_intrinsic, n);
	
	int ret = p.compute_pose(R,t);
	
	Eigen::Quaterniond q(R);
	
	pose_mat << t,q.vec();
	
	return ret;
}


int CPnP::CPnP_lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    Eigen::Matrix3d R;
	Eigen::Vector3d t;
	
	Eigen::MatrixXd obj_pts_=obj_pts.array().transpose(), img_pts_=img_pts.array().transpose();
    
    mrpt::vision::lhm l(obj_pts_, img_pts_, cam_intrinsic, n);
    
    int ret = l.compute_pose(R,t);
    
    Eigen::Quaterniond q(R);
    
    pose_mat<<t,q.vec();
    
    return ret;
    
}
