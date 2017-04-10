// SO3 - PnP

#include <mrpt/utils/types_math.h> // Eigen must be included first via MRPT to enable the plugin system

namespace mrpt
{
    namespace vision
    {
        namespace pnp
        {
            /** \addtogroup SO(3) pnp Perspective-n-Point pose estimation
             *  \ingroup mrpt_vision_grp
             *  @{
             */

            /**
             * @class so3
             * @author Chandra Mangipudi
             * @date 12/08/16
             * @file upnp.h
             * @brief SO(3) PnP
             */
            class so3
            {
                public:

                    //! Constructor for so3 class
                    so3(const Eigen::MatrixXd& obj_pts_, const Eigen::MatrixXd& img_pts_, const Eigen::MatrixXd& cam_intrinsic_, int n0);

                    /**
                     * @brief Function to compute pose
                     * @param[out] R_ Rotation Matrix
                     * @param[out] t_ Translation Vector
                     * @return success flag
                     */
                    bool compute_pose(Eigen::Matrix3d& R_, Eigen::Vector3d& t_);

                    /**
                     * @brief Quaternion to rotation matrix
                     * @param Vec Vector component of a quaternion
                     * @return Rotation Matrix
                     */
                    Eigen::Matrix3d  quatrot(Eigen::Vector3d  Vec);

                    /**
                     * @brief Function to calculate error
                     * @param[in] R Current estimate of Rotation matrix
                     * @param[in] flag Update translation vector if flag is True
                     * @param[out] err Error due to current pose
                     * @param[out] Vec Update quaternion which reduces the error
                     * @param[out] rgm Updated translation vector if flag is True
                     */
                    void err_calc(Eigen::Matrix3d & R, int flag, Eigen::VectorXd & err, Eigen::Vector3d & Vec, Eigen::Vector3d & rgm);

                    /**
                     * @brief Helper function to @compute_pose
                     * @param[in] R_guess Initial estimate of Rotation Matrix
                     */
                    void findPosSO3(Eigen::Matrix3d & R_guess);

                    /**
                     * @brief Rotation matrix to Quaternion conversion function
                     * @param[in] R Rotation matrix
                     * @return Quaternion
                     */
                    Eigen::Vector4d  dcm2quat(Eigen::Matrix3d  R);

                    private:

                    Eigen::MatrixXd  cam_intrinsic; //! Camera intrinsic matrix
                    Eigen::MatrixXd  obj_pts; //! Object points
                    Eigen::MatrixXd  img_pts; //! Image points
                    Eigen::MatrixXd  beta1, Pr1, Pr; //! Internal member variables

                    Eigen::Matrix3d  Ra, Rc, R_ret, R; //! Internal Rotation matrices
                    Eigen::Matrix3d  step, N; //! Internal 3x3 variables
                    Eigen::Vector3d  rgm, Veca, Vecc,  dummyrgm, ax; //! Internal 3x1 vectors
                    Eigen::VectorXd  err; //! Error nx1 vector
                    Eigen::VectorXd  gam; //! Internal nx1 vector

                    double error_a, error_c;//! error norms
                    double stepsize; //! angle by which to update rotation matrix
                    double ang,c_ang,s_ang; //! Internal variables

                    int k3, k1, k2, k; //! Variables to keep track of number of iterations
                    int n; //! Number of 2d/3d correspondences

            };
            /** @}  */ // end of grouping
        }
    }
}
