/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/vision/utils.h>
#include <mrpt/vision/pnp_algos.h>


// Opencv 2.3 had a broken <opencv/eigen.h> in Ubuntu 14.04 Trusty => Disable PNP classes
#include <mrpt/config.h>

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM<0x240
#	undef MRPT_HAS_OPENCV
#	define MRPT_HAS_OPENCV 0
#endif

#include <iostream>

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Core>
#include <Eigen/Dense>

#include <mrpt/otherlibs/do_opencv_includes.h>
#if MRPT_HAS_OPENCV
#	include <opencv2/core/eigen.hpp>
#endif

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
    try{
        #if MRPT_HAS_OPENCV==1

        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R_eig;
        Eigen::MatrixXd t_eig;

        // Compute pose
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
		throw(-1);
        #endif
    }
    catch(int e)
    {
        switch(e)
        {
            case -1: std::cout << "Please install OpenCV for DLS-PnP" << std::endl;
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}

bool mrpt::vision::pnp::CPnP::epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    try{
        #if MRPT_HAS_OPENCV==1

        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R_eig;
        Eigen::MatrixXd t_eig;

        // Compute pose
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
        throw(-1);
        #endif
    }
    catch(int e){
        switch(e)
        {
            case -1: std::cout << "Please install OpenCV for DLS-PnP" << std::endl;
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}

bool mrpt::vision::pnp::CPnP::upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    try{
        #if MRPT_HAS_OPENCV==1

        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R_eig;
        Eigen::MatrixXd t_eig;

        // Compute pose
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
        throw(-1);
        #endif
    }
    catch(int e)
    {
        switch(e)
        {
            case -1: std::cout << "Please install OpenCV for DLS-PnP" << std::endl;
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}


bool mrpt::vision::pnp::CPnP::p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){

    try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::p3p p(cam_in_eig);
        bool ret = p.solve(R,t, obj_pts_eig, img_pts_eig);

        Eigen::Quaterniond q(R);

        pose_mat << t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}


bool mrpt::vision::pnp::CPnP::rpnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose();
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts;
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::rpnp r(obj_pts_eig, img_pts_eig, cam_in_eig, n);
        bool ret = r.compute_pose(R,t);

        Eigen::Quaterniond q(R);

        pose_mat << t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}

bool mrpt::vision::pnp::CPnP::ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
	try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose();
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts;
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::ppnp p(obj_pts_eig,img_pts_eig, cam_in_eig);

        bool ret = p.compute_pose(R,t,n);

        Eigen::Quaterniond q(R);

        pose_mat << t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}

bool mrpt::vision::pnp::CPnP::posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
	try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::posit p(obj_pts_eig,img_pts_eig, cam_in_eig, n);

        bool ret = p.compute_pose(R,t);

        Eigen::Quaterniond q(R);

        pose_mat << t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }

}

bool mrpt::vision::pnp::CPnP::lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose();
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts;
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::lhm l(obj_pts_eig, img_pts_eig, cam_intrinsic, n);

        bool ret = l.compute_pose(R,t);

        Eigen::Quaterniond q(R);

        pose_mat<<t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}

bool mrpt::vision::pnp::CPnP::so3(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
{
    try{
        // Input 2d/3d correspondences and camera intrinsic matrix
        Eigen::MatrixXd cam_in_eig,img_pts_eig, obj_pts_eig;

        // Check for consistency of input matrix dimensions
        if (img_pts.rows() != obj_pts.rows() || img_pts.cols() !=obj_pts.cols())
            throw(2);
        else if (cam_intrinsic.rows()!=3 || cam_intrinsic.cols()!=3)
            throw(3);

        if(obj_pts.rows() < obj_pts.cols())
        {
            cam_in_eig=cam_intrinsic.transpose();
            img_pts_eig=img_pts.transpose().block(0,0,n,2);
            obj_pts_eig=obj_pts.transpose();
        }
        else
        {
            cam_in_eig=cam_intrinsic;
            img_pts_eig=img_pts.block(0,0,n,2);
            obj_pts_eig=obj_pts;
        }

        // Output pose
        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        // Compute pose
        mrpt::vision::pnp::p3p p(cam_in_eig);
        p.solve(R,t, obj_pts_eig, img_pts_eig);

        mrpt::vision::pnp::so3 s(obj_pts_eig, img_pts_eig, cam_in_eig, n);
        bool ret = s.compute_pose(R,t);

        Eigen::Quaterniond q(R);

        pose_mat<<t,q.vec();

        return ret;
    }
    catch(int e)
    {
        switch(e)
        {
            case  2: std::cout << "2d/3d correspondences mismatch\n Check dimension of obj_pts and img_pts" << std::endl;
            case  3: std::cout << "Camera intrinsic matrix does not have 3x3 dimensions " << std::endl;
        }
        return false;
    }
}
