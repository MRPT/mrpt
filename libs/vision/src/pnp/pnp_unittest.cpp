#include <gtest/gtest.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <mrpt/vision/pnp_algos.h>

class CPnPTest: public::testing::Test
{
    public:
        mrpt::vision::pnp::CPnP cpnp;
        
        Eigen::MatrixXd obj_pts, img_pts, pose_est;
        Eigen::Matrix3d R, I3, R_est;
        Eigen::Vector3d t, t_est;
        int n;

        virtual void SetUp()
        {
            n=6;
            
            obj_pts = Eigen::MatrixXd::Zero(3,n);
            img_pts = Eigen::MatrixXd::Zero(3,n);
            obj_pts << 0, 20,  10, 15, 14, 16,
                        0, 0, -10, -5, -2, -5,
                        0, 40,   0, 24, 21, 13;
                        
            R << -0.3536, 0.3536, -0.8660,
                  0.9330, 0.0670, -0.3536,
                 -0.0670, -0.9330, -0.3536;

            t << 20, -30, 100;
    
            for (int i = 0; i < n; i++)
            {
                img_pts.col(i) = (R * obj_pts.col(i) + t);
                img_pts.col(i) /= img_pts(2,i);
            }
            
            I3 = Eigen::MatrixXd::Identity(3, 3);
            
            pose_est = Eigen::MatrixXd::Zero(6,1);
                     
        }
        virtual void TearDown()
        {
            
        }
    
    
    
};

TEST_F(CPnPTest, p3p_TEST)
{
    cpnp.CPnP_p3p(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, rpnp_TEST)
{
    cpnp.CPnP_rpnp(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, ppnp_TEST)
{
    cpnp.CPnP_ppnp(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, posit_TEST)
{
    cpnp.CPnP_posit(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
}

TEST_F(CPnPTest, lhm_TEST)
{
    cpnp.CPnP_lhm(obj_pts, img_pts, n, I3, pose_est);
    
    t_est<<pose_est(0), pose_est(1), pose_est(2);
    
    double err_t = (t-t_est).norm();
    
    EXPECT_LE(err_t, 2);
}

#if MRPT_HAS_OPENCV

    TEST_F(CPnPTest, dls_TEST)
    {
        cpnp.CPnP_dls(obj_pts, img_pts, n, I3, pose_est);
        
        t_est<<pose_est(0), pose_est(1), pose_est(2);
        
        double err_t = (t-t_est).norm();
        
        EXPECT_LE(err_t, 2);
    }
    
    TEST_F(CPnPTest, epnp_TEST)
    {
        cpnp.CPnP_epnp(obj_pts, img_pts, n, I3, pose_est);
        
        t_est<<pose_est(0), pose_est(1), pose_est(2);
        
        double err_t = (t-t_est).norm();
        
        EXPECT_LE(err_t, 2);
    }
    
    TEST_F(CPnPTest, DISABLED_upnp_TEST)
    {
        cpnp.CPnP_upnp(obj_pts, img_pts, n, I3, pose_est);
        
        t_est<<pose_est(0), pose_est(1), pose_est(2);
        
        double err_t = (t-t_est).norm();
        
        EXPECT_LE(err_t, 2);
    }

#endif