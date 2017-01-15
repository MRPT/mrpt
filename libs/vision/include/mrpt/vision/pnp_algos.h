/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef __pnp_algos_h
#define __pnp_algos_h

#include <mrpt/config.h>

#include <mrpt/vision/link_pragmas.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace mrpt
{
    namespace vision
    {
        /** Perspective n Point (PnP) Algorithms toolkit for MRPT \ingroup mrpt_vision_grp */
        namespace pnp
        {
            /** \addtogroup pnp Perspective-n-Point pose estimation
             *  \ingroup mrpt_vision_grp
             *  @{  
             */
             
            /**
             * @class CPnP
             * @author Chandra Mangipudi
             * @date 17/08/16
             * @file pnp_algos.h
             * @brief PnP Algorithms toolkit for MRPT
             */
 
            /** This class is used for Pose estimation from a known landmark using a monocular camera.
             *  The toolkit comprises of state of the art  PnP (Perspective n Point) algorithms
             * 
             *  The Python Bindings pnp_perf_comp.py can be used to generate performance comparison using 
             *  standard tests between the different algorithms.
             * 
             *  <h2> <a href="pnp_perf_comp.html">Performance comparison Results </a> </h2>
             *  
             */
            class VISION_IMPEXP CPnP
            {
                public:
                
                    /**
                     * @brief \cite hesch Direct Least Squares (DLS) - PnP : Algorithm formulates position as a function of rotation. 
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
                    bool dls(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

                    /**
                     * @brief \cite lepetit Efficient-PnP (EPnP) - Algorithm takes 4 control points based on n points and uses the control points to compute the pose
                     * 
                     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 (only 4 points used) array [p_x, p_y, p_z]
                     * @param[in] img_pts Image points in pixels nX3 (only 4 points used) array containing pixel data from camera [u, v, 1]
                     * @param[in] n number of 2D-3D correspondences
                     * @param[in] cam_intrinsic Camera Intrinsic matrix
                     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
                     * @return success flag
                     */
                    bool epnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

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
                    bool upnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
                    
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
                    bool p3p(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
                    
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
                    bool rpnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
                    
                    /**
                     * @brief \cite garro Procrustes-PnP(PPnP) Algorithm : Iterative SVD based algorithm
                     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 array [p_x, p_y, p_z]
                     * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
                     * @param[in] n number of 2D-3D correspondences
                     * @param[in] cam_intrinsic Camera Intrinsic matrix
                     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
                     * @return success flag
                     */
                    bool ppnp(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

                    /**
                     * @brief \cite dementhon Pose from Orthogoanlity and Scaling :Iterative (POSIT) -  A Geometric algorithm to compute scale and orthogonality independently
                     * @param[in] obj_pts Object points in Camera Co-ordinate system {C} nX3 array [p_x, p_y, p_z]
                     * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
                     * @param[in] n number of 2D-3D correspondences
                     * @param[in] cam_intrinsic Camera Intrinsic matrix
                     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
                     * @return success flag
                     */
                    bool posit(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
            
                    /**@brief \cite lu Lu-Hager-Mjolsness(LHM)-PnP algorithm : Iterative algorithm to reduce object space error
                     * @param[in] obj_pts   Object points in Camera Co-ordinate system {C} nX3 array [p_x p_y p_z]
                     * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
                     * @param[in] n number of 2D-3D correspondences
                     * @param[in] cam_intrinsic Camera Intrinsic matrix
                     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
                     * @return success flag
                     */
                    bool lhm(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
                    
                    /**
                     * @brief \cite chandra SO(3) - PnP: Gradient descent based local search optimization
                     * @param[in] obj_pts   Object points in Camera Co-ordinate system {C} nX3 array [p_x p_y p_z]
                     * @param[in] img_pts Image points in pixels nX3 array containing pixel data from camera [u, v, 1]
                     * @param[in] n number of 2D-3D correspondences
                     * @param[in] cam_intrinsic Camera Intrinsic matrix
                     * @param[out] pose_mat Output pose vector 6X1, pose_mat[0:2]-> Translation, pose_mat[3:5] -> Quaternion vector component 
                     * @return success flag
                     * @return 
                     */
                    bool so3(const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);
            };
            
            /** @}  */ // end of grouping
        }
    }
}

#endif
