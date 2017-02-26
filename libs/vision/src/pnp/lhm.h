/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#ifndef LHM_

#define LHM_

#include <iostream>

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>


#define TOL_LHM 0.00001
#define EPSILON_LHM 0.00000001

namespace mrpt
{
    namespace vision
    {
        namespace pnp
        {
            /** \addtogroup pnp Perspective-n-Point pose estimation
             *  \ingroup mrpt_vision_grp
             *  @{  
             */
             
            /**
             * @class lhm
             * @author Chandra Mangipudi
             * @date 10/08/16
             * @file lhm.h
             * @brief Lu Hage Mjolsness - Iterative PnP Algorithm (Eigen Implementation
             */
            class lhm
            {

                Eigen::MatrixXd obj_pts; //! Object points in Camera Co-ordinate system
                Eigen::MatrixXd img_pts; //! Image points in pixel co-ordinates
                Eigen::MatrixXd cam_intrinsic; //! Camera intrinsic matrix
                Eigen::MatrixXd P; //! Trnaspose of Object points @obj_pts
                Eigen::MatrixXd Q; //! Transpose of Image points @img_pts

                Eigen::Matrix3d R; //! Matrix for internal computations
                Eigen::Matrix3d G; //! Rotation Matrix 
                Eigen::Vector3d t; //! Translation Vector

                std::vector<Eigen::Matrix3d> F; //! Storage matrix for each point
                double err;  //! Error variable for convergence selection
                double err2; //! Error variable for convergence selection

                int n; //! Number of 2d/3d correspondences

            public:

                //! Constructor for the LHM class
                lhm(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0);
                //~lhm();
                
                /**
                 * @brief Function to compute pose using LHM PnP algorithm
                 * @param R_ Rotation matrix
                 * @param t_ Trnaslation Vector
                 * @return Success flag
                 */
                bool compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);

                /**
                 * @brief Function to compute pose during an iteration
                 */
                void absKernel();

                /**
                 * @brief Function to estimate translation given an estimated rotation matrix
                 */
                void estimate_t();

                /**
                 * @brief Transform object points in Body frame (Landmark Frame) to estimated Camera Frame
                 */
                void xform();

                /**
                 * @brief Iternal Function of quaternion
                 * @param q Quaternion
                 * @return 
                 */
                Eigen::Matrix4d qMatQ(Eigen::VectorXd q);

                /**
                 * @brief Iternal Function of quaternion
                 * @param q Quaternion
                 * @return 
                 */
                Eigen::Matrix4d qMatW(Eigen::VectorXd q);
            };
            
            /** @}  */ // end of grouping
            
        }
    }
}
#endif




