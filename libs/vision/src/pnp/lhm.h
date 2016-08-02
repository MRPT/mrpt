#ifndef LHM_

#define LHM_

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>


#define TOL_LHM 0.00001
#define EPSILON_LHM 0.00000001

namespace mrpt
{
    namespace vision
    {
        class lhm
        {

            Eigen::MatrixXd obj_pts, img_pts, cam_intrinsic, P, Q;

            Eigen::Matrix3d R, G;
            Eigen::Vector3d t;

            std::vector<Eigen::Matrix3d> F;
            double err, err2;

            int n;

        public:

            lhm(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd cam_, int n0);
            //~lhm();

            bool compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);

            void absKernel();

            void estimate_t();

            void xform();

            Eigen::Matrix4d qMatQ(Eigen::VectorXd q);

            Eigen::Matrix4d qMatW(Eigen::VectorXd q);
        };
    }
}
#endif




