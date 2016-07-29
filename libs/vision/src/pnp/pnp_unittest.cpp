#include <gtest/gtest.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <mrpt/vision/pnp_algos.h>


TEST(CPnP_Test, lhm_ALGO_TEST)
{
    int n = 6;

	Eigen::Matrix3d R(3, 3);
	Eigen::Vector3d t;
	R << -0.3536, 0.3536, -0.8660,
		0.9330, 0.0670, -0.3536,
		-0.0670, -0.9330, -0.3536;

	t << 20, -30, 100;

	Eigen::MatrixXd obj_pts(3, n), img_pts(3, n);

    obj_pts << 20, 0,  10, 15, 14, 16,
                0, 0, -10, -5, -2, -5,
               40, 0,   0, 24, 21, 13;

	for (int i = 0; i < n; i++)
	{
		img_pts.col(i) = (R * obj_pts.col(i) + t);
		img_pts.col(i) /= img_pts(2,i);
	}

	Eigen::Matrix3d R_est;
	Eigen::Vector3d t_est;
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    
    mrpt::vision::pnp::CPnP p;

    Eigen::MatrixXd pose_est(6,1);
    p.CPnP_lhm(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
    
}

TEST(CPnP_Test, p3p_ALGO_TEST)
{
    int n = 6;

	Eigen::Matrix3d R(3, 3);
	Eigen::Vector3d t;
	R << -0.3536, 0.3536, -0.8660,
		0.9330, 0.0670, -0.3536,
		-0.0670, -0.9330, -0.3536;

	t << 20, -30, 100;

	Eigen::MatrixXd obj_pts(3, n), img_pts(3, n);

    obj_pts << 20, 0,  10, 15, 14, 16,
                0, 0, -10, -5, -2, -5,
               40, 0,   0, 24, 21, 13;

	for (int i = 0; i < n; i++)
	{
		img_pts.col(i) = (R * obj_pts.col(i) + t);
		img_pts.col(i) /= img_pts(2,i);
	}

	Eigen::Matrix3d R_est;
	Eigen::Vector3d t_est;
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    
    mrpt::vision::pnp::CPnP p;

    Eigen::MatrixXd pose_est(6,1);
    p.CPnP_p3p(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 0.5);
    
}

TEST(CPnP_Test, ppnp_ALGO_TEST)
{
    int n = 6;

	Eigen::Matrix3d R(3, 3);
	Eigen::Vector3d t;
	R << -0.3536, 0.3536, -0.8660,
		0.9330, 0.0670, -0.3536,
		-0.0670, -0.9330, -0.3536;

	t << 20, -30, 100;

	Eigen::MatrixXd obj_pts(3, n), img_pts(3, n);

	obj_pts << 20, 0,  10, 15, 14, 16,
                0, 0, -10, -5, -2, -5,
               40, 0,   0, 24, 21, 13;

	for (int i = 0; i < n; i++)
	{
		img_pts.col(i) = (R * obj_pts.col(i) + t);
		img_pts.col(i) /= img_pts(2,i);
	}

	Eigen::Matrix3d R_est;
	Eigen::Vector3d t_est;
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    
    mrpt::vision::pnp::CPnP p;

    Eigen::MatrixXd pose_est(6,1);
    p.CPnP_ppnp(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 0.5);
    
}

TEST(CPnP_Test, rpnp_ALGO_TEST)
{
    int n = 6;

	Eigen::Matrix3d R(3, 3);
	Eigen::Vector3d t;
	R << -0.3536, 0.3536, -0.8660,
		0.9330, 0.0670, -0.3536,
		-0.0670, -0.9330, -0.3536;

	t << 20, -30, 100;

	Eigen::MatrixXd obj_pts(3, n), img_pts(3, n);

	obj_pts << 20, 0,  10, 15, 14, 16,
                0, 0, -10, -5, -2, -5,
               40, 0,   0, 24, 21, 13;

	for (int i = 0; i < n; i++)
	{
		img_pts.col(i) = (R * obj_pts.col(i) + t);
		img_pts.col(i) /= img_pts(2,i);
	}

	Eigen::Matrix3d R_est;
	Eigen::Vector3d t_est;
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    
    mrpt::vision::pnp::CPnP p;

    Eigen::MatrixXd pose_est(6,1);
    p.CPnP_rpnp(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 0.5);
    
}

TEST(CPnP_Test, posit_ALGO_TEST)
{
    int n = 6;

	Eigen::Matrix3d R(3, 3);
	Eigen::Vector3d t;
	R << -0.3536, 0.3536, -0.8660,
		0.9330, 0.0670, -0.3536,
		-0.0670, -0.9330, -0.3536;

	t << 20, -30, 100;

	Eigen::MatrixXd obj_pts(3, n), img_pts(3, n), i_pts(2,n);

    obj_pts << 0, 20,  10, 15, 14, 16,
               0, 0, -10, -5, -2, -5,
               0, 40,   0, 24, 21, 13;

	for (int i = 0; i < n; i++)
	{
		img_pts.col(i) = (R * obj_pts.col(i) + t);
		img_pts.col(i) /= img_pts(2,i);
	}
    
    i_pts=img_pts.block(0,0,2,n);
    
	Eigen::Matrix3d R_est;
	Eigen::Vector3d t_est;
    
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    
    mrpt::vision::pnp::CPnP p;
    
    Eigen::MatrixXd pose_est(6,1);
    p.CPnP_posit(obj_pts, i_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
    
}

#if MRPT_HAS_OPENCV

    TEST(CPnP_Test, epnp_ALGO_TEST)
    {
        int n = 6;

        Eigen::Matrix3d R(3, 3);
        Eigen::Vector3d t;
        R << -0.3536, 0.3536, -0.8660,
            0.9330, 0.0670, -0.3536,
            -0.0670, -0.9330, -0.3536;

        t << 20, -30, 100;

        Eigen::MatrixXd obj_pts(3, n), img_pts(3, n), i_pts(2,n);

        obj_pts << 20, 0,  10, 15, 14, 16,
                    0, 0, -10, -5, -2, -5,
                   40, 0,   0, 24, 21, 13;

        for (int i = 0; i < n; i++)
        {
            img_pts.col(i) = (R * obj_pts.col(i) + t);
            img_pts.col(i) /= img_pts(2,i);
        }

        Eigen::Matrix3d R_est;
        Eigen::Vector3d t_est;
        
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        
        mrpt::vision::pnp::CPnP p;
        
        i_pts= img_pts.block(0,0,2,n);

        Eigen::MatrixXd pose_est(6,1);
        p.CPnP_epnp(obj_pts, i_pts, n, I3, pose_est);
        
        t_est<<pose_est(0), pose_est(1), pose_est(2);
        
        double err_t = (t-t_est).norm();
        
        EXPECT_LE(err_t, 2);
        
    }

    TEST(CPnP_Test, dls_ALGO_TEST)
    {
        int n = 6;

        Eigen::Matrix3d R(3, 3);
        Eigen::Vector3d t;
        R << -0.3536, 0.3536, -0.8660,
            0.9330, 0.0670, -0.3536,
            -0.0670, -0.9330, -0.3536;

        t << 20, -30, 100;

        Eigen::MatrixXd obj_pts(3, n), img_pts(3, n), i_pts(2,n);

        obj_pts << 20, 0,  10, 15, 14, 16,
                    0, 0, -10, -5, -2, -5,
                   40, 0,   0, 24, 21, 13;

        for (int i = 0; i < n; i++)
        {
            img_pts.col(i) = (R * obj_pts.col(i) + t);
            img_pts.col(i) /= img_pts(2,i);
        }

        Eigen::Matrix3d R_est;
        Eigen::Vector3d t_est;
        
        Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
        
        mrpt::vision::pnp::CPnP p;
        
        i_pts= img_pts.block(0,0,2,n);

        Eigen::MatrixXd pose_est(6,1);
        p.CPnP_dls(obj_pts, i_pts, n, I3, pose_est);
        
        t_est<<pose_est(0), pose_est(1), pose_est(2);
        
        double err_t = (t-t_est).norm();
        
        EXPECT_LE(err_t, 0.5);
        
    }
    
#endif