#include <iostream>
#include <Eigen/Dense>

namespace mrpt
{
    namespace vision
    {

        class ppnp
        {
            public:
            
            ppnp(const Eigen::MatrixXd& obj_pts, const Eigen::MatrixXd& img_pts, const Eigen::MatrixXd& cam_intrinsic);
            ~ppnp();
            
            bool compute_pose(Eigen::Matrix3d& R, Eigen::VectorXd& t, int n);
            
            private:
            
            Eigen::MatrixXd P, S,C;
            
        };
    }
}
