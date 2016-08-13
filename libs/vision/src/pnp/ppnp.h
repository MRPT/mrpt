#include <iostream>
#include <Eigen/Dense>

namespace mrpt
{
    namespace vision
    {
        /** \addtogroup pnp Perspective-n-Point pose estimation
         *  \ingroup mrpt_vision_grp
         *  @{  
         */
         
        /**
         * @class ppnp
         * @author Chandra Mangipudi
         * @date 10/08/16
         * @file ppnp.h
         * @brief Procrustes - PnP 
         */
        class ppnp
        {
        public:
            
            //! Constructor for the P-PnP class
            ppnp(const Eigen::MatrixXd& obj_pts, const Eigen::MatrixXd& img_pts, const Eigen::MatrixXd& cam_intrinsic);
            //! Standard Default destructor for P-PnP class
            ~ppnp();
            
            /**
             * @brief Function to compute pose 
             * @param[out] R Rotation matrix
             * @param t Trnaslation Vector
             * @param n Number of 2d/3d correspondences
             * @return 
             */
            bool compute_pose(Eigen::Matrix3d& R, Eigen::VectorXd& t, int n);
            
            private:
            
            Eigen::MatrixXd P; //! Image points in pixels
            Eigen::MatrixXd S; //! Object points in Camera Co-ordinate system
            Eigen::MatrixXd C; //! Camera intrinsic matrix
            
        };
        
        /** @}  */ // end of grouping
    }
}
