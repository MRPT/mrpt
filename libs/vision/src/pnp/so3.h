// SO3 - PnP 
#include <iostream>
#include <Eigen/Dense>

namespace mrpt
{
    namespace vision
    {

        class so3
        {
            public:
            
            so3(const Eigen::MatrixXd& obj_pts_, const Eigen::MatrixXd& img_pts_, const Eigen::MatrixXd& cam_intrinsic_, int n0);
            
            bool compute_pose(Eigen::Matrix3d& R_, Eigen::Vector3d& t_);
            
            Eigen::Matrix3d  quatrot(Eigen::Vector3d  Vec);
            
            void err_calc(Eigen::Matrix3d & R, int flag, Eigen::VectorXd & err, Eigen::Vector3d & Vec, Eigen::Vector3d & rgm);
            
            void findPosSO3(Eigen::Matrix3d & R_guess);
            
            Eigen::Vector4d  dcm2quat(Eigen::Matrix3d  R);
            
            private:
            
            Eigen::MatrixXd  cam_intrinsic, obj_pts, img_pts;
            Eigen::MatrixXd beta1, Pr1, Pr;
            
            Eigen::Matrix3d  Ra, Rc, R_ret, R, step, N;
            Eigen::Vector3d  rgm, Veca, Vecc,  dummyrgm, ax;
            Eigen::VectorXd  err, gam;
            
            double error_a, error_c, stepsize;
            double ang,c_ang,s_ang;
            
            int k3, k1, k2, k, n;
            
        };
    }
}