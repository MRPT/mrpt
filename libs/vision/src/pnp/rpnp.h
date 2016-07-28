#ifndef rpnp_h
#define rpnp_h

#include <Eigen/Dense>

namespace mrpt
{
    namespace vision
    {
        
        class rpnp
        {
            Eigen::MatrixXd obj_pts, img_pts, cam_intrinsic, P, Q;

            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            int n;

        public:
            rpnp(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0);
            bool compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);
            void getp3p(double l1, double l2, double A5, double C1, double C2, double D1, double D2, double D3, Eigen::VectorXd& vec);
            Eigen::VectorXd getpoly7(const Eigen::VectorXd& vin);
            void calcampose(Eigen::MatrixXd& XXc, Eigen::MatrixXd& XXw, Eigen::Matrix3d& R2, Eigen::Vector3d& t2);
        };
        
    }
    
}

#endif