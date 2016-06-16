#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <iostream>

#define LOOP_MAX_COUNT 30

class POSIT
{
	
	// Inputs
	Eigen::MatrixXd obj_pts, img_pts, cam_intrinsic, obj_matrix; 
	double f;
	Eigen::VectorXd epsilons;
	int n;
	
	// Outputs
	Eigen::MatrixXd R;
	Eigen::VectorXd t; 
	
	Eigen::MatrixXd obj_vecs, img_vecs, img_vecs_old;
	
	public:
	
	POSIT(Eigen::MatrixXd obj_pts_, Eigen::MatrixXd img_pts_, Eigen::MatrixXd camera_intrinsic_, int n);
	
	
	void POS();
	int compute_pose(Eigen::Ref<Eigen::Matrix3d> R_, Eigen::Ref<Eigen::Vector3d> t_);
	long get_img_diff();
	
};


