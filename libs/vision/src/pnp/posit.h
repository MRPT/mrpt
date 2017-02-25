/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <iostream>

#define LOOP_MAX_COUNT 30

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
             * @class POSIT
             * @author Chandra Mangipudi
             * @date 10/08/16
             * @file posit.h
             * @brief Pose from Orthogonality and Scaling (POSIT) - Eigen Implementation
             */
            class posit
            {
                
                Eigen::MatrixXd obj_pts; //! Object Points in Camera Co-ordinate system
                Eigen::MatrixXd img_pts; //! Image Points in pixels
                Eigen::MatrixXd cam_intrinsic; //! Camera Intrinsic matrix
                Eigen::MatrixXd obj_matrix; //! Pseudo-Inverse of Object Points matrix
                
                double f; //! Focal Length from camera intrinsic matrix
                Eigen::VectorXd epsilons; //! Co-efficients used for scaling
                int n; //! Number of 2d/3d correspondences
                
                Eigen::MatrixXd R; //! Rotation Matrix
                Eigen::VectorXd t; //! Translation Vector
                
                Eigen::MatrixXd obj_vecs; //! Object Points relative to 1st object point
                Eigen::MatrixXd img_vecs; //! Image Points relative to 1st image point
                Eigen::MatrixXd img_vecs_old; //! Used to store img_vecs from previous iteration 
                
            public:
                
                //! Constructor for P-PnP class
                posit(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd camera_intrinsic_, int n);
                
                /**
                 * @brief Function used to compute pose from orthogonality
                 */
                void POS();
                
                /**
                 * @brief Computes pose using iterative computation of @func POS()
                 * @param[out] R_
                 * @param[out] t_
                 * @return true on success
                 */
                bool compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);
                
                /**
                 * @brief Function to check for convergence
                 * @return Representative value of error
                 */
                long get_img_diff();
                
            };
            
            /** @}  */ // end of grouping
            
        }
    }
}

