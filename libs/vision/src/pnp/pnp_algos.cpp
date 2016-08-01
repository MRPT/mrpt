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

#if MRPT_HAS_OPENCV

    /**
     * @brief Direct Least Squares (DLS) - PnP : Algorithm formulates position as a function of rotation. 
     *        Use Cayley's rotation theorem to represent the rotation using parameters ($s_1, s_2, s_3$).
     *        Solve the rotation using multi-variate polynomial expressions
     *        
     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
     * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
     * @param[in] n number of 2D-3D correspondences
     * @param[in] cam_intrinsic Camera Intrinsic matrix
     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
     * @return success flag
     */
    int CPnP::CPnP_dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
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

    /**
     * @brief Efficient-PnP (EPnP) - Algorithm takes 4 control points based on n points and uses the control points to compute the pose
     * 
     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
     * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
     * @param[in] n number of 2D-3D correspondences
     * @param[in] cam_intrinsic Camera Intrinsic matrix
     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
     * @return success flag
     */
    int CPnP::CPnP_epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
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

    /**
     * @brief \cite Kneip2014 Unified-PnP (UPnP) : Algorithm to compute pose from unknown camera intrinsic matrix
     * 
     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
     * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
     * @param[in] n number of 2D-3D correspondences
     * @param[in] cam_intrinsic Camera Intrinsic matrix
     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
     * @return success flag
     */
    int CPnP::CPnP_upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
        
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

/**
 * @brief \cite kneip P3P - A closed form solution for n=3, 2D-3D correspondences
 * 
 * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
 * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
 * @param[in] n number of 2D-3D correspondences
 * @param[in] cam_intrinsic Camera Intrinsic matrix
 * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
 * @return success flag
 */
int CPnP::CPnP_p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::p3p p(cam_in_eig);
    int ret = p.solve(R_eig,t_eig, obj_pts_eig, img_pts_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}

/**
 * @brief \cite xie Robust (R-PnP)- A closed form solution with intermediate P3P computations
 * 
 * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
 * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
 * @param[in] n number of 2D-3D correspondences
 * @param[in] cam_intrinsic Camera Intrinsic matrix
 * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
 * @return success flag
 */
int CPnP::CPnP_rpnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat){
    
    Eigen::MatrixXd cam_in_eig=cam_intrinsic.transpose(), img_pts_eig=img_pts.transpose(), obj_pts_eig=obj_pts.transpose();
    Eigen::Matrix3d R_eig; 
    Eigen::Vector3d t_eig;
    
    mrpt::vision::rpnp r(obj_pts_eig, img_pts_eig, cam_in_eig, n);
    int ret = r.compute_pose(R_eig,t_eig);
    
    Eigen::Quaterniond q(R_eig);
    
    pose_mat << t_eig,q.vec();
    
    return ret;
}

/**
 * @brief \cite garro Procrustes-PnP(PPnP) Algorithm : Iterative SVD based algorithm
 * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 array [p_x, p_y, p_z]
 * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
 * @param[in] n number of 2D-3D correspondences
 * @param[in] cam_intrinsic Camera Intrinsic matrix
 * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
 * @return success flag
 */
int CPnP::CPnP_ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
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

/**
 * @brief \cite dementhon Pose from Orthogoanlity and Scaling :Iterative (POSIT) -  A Geometric algorithm to compute scale and orthogonality independently
 * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 array [p_x, p_y, p_z]
 * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
 * @param[in] n number of 2D-3D correspondences
 * @param[in] cam_intrinsic Camera Intrinsic matrix
 * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
 * @return success flag
 */
int CPnP::CPnP_posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
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

/**@brief \cite lu Lu-Hager-Mjolsness(LHM)-PnP algorithm : Iterative algorithm to reduce object space error
 * @param[in] obj_pts   Object points in Camera Co-ordinate system {C} nX3 array [p_x p_y p_z]
 * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
 * @param[in] n number of 2D-3D correspondences
 * @param[in] cam_intrinsic Camera Intrinsic matrix
 * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
 * @return success flag
 */
int CPnP::CPnP_lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat)
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
