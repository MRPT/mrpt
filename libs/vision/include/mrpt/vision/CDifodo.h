/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDifodo_H
#define CDifodo_H

#include <mrpt/utils/types_math.h> // Eigen
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/vision/link_pragmas.h>
//#include <unsupported/Eigen/MatrixFunctions>

namespace mrpt
{
	namespace vision
	{
		/** This abstract class implements a method called "Difodo" to perform Visual odometry with range cameras.
		  *	It is based on the range flow equation and assumes that the scene is rigid.
		  *	It can work with different image resolutions (640 x 480, 320 x 240 or 160 x 120) and a different number of
		  * coarse-to-fine levels which can be adjusted with the member variables (rows,cols,ctf_levels).
		  *
		  * How to use:
		  * 	- A derived class must be created which defines the method "loadFrame(...)" according to the user application.
		  *		  This method has to load the depth image into the variable "depth_wf".
		  *		- Call loadFrame();
		  *		- Call odometryCalculation();
		  *
		  *	For further information have a look at the apps:
		  *    - [DifOdometry-Camera](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-camera/)
		  *    - [DifOdometry-Datasets](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-datasets/)
		  *
		  *	Please refer to the respective publication when using this method: 
		  *		title = {Fast Visual Odometry for {3-D} Range Sensors},
		  *		author = {Jaimez, Mariano and Gonzalez-Jimenez, Javier},
		  *		journal = {IEEE Transactions on Robotics},
		  *		volume = {31},
		  *		number = {4},
		  *		pages = {809 - 822},
		  *		year = {2015}
		  *
		  * - JUN/2013: First design.
		  * - JAN/2014: Integrated into MRPT library.
		  * - DIC/2014: Reformulated and improved. The class now needs Eigen version 3.1.0 or above.
		  *
		  *  \sa CDifodoCamera, CDifodoDatasets
		  *  \ingroup mrpt_vision_grp
		  */

		class VISION_IMPEXP CDifodo {
		protected:

			/** Matrix that stores the original depth frames with the image resolution */
			Eigen::MatrixXf depth_wf;

			/** Matrices that store the point coordinates after downsampling. */
			std::vector<Eigen::MatrixXf> depth;
			std::vector<Eigen::MatrixXf> depth_old;
			std::vector<Eigen::MatrixXf> depth_inter;
			std::vector<Eigen::MatrixXf> depth_warped;
			std::vector<Eigen::MatrixXf> xx;
			std::vector<Eigen::MatrixXf> xx_inter;
			std::vector<Eigen::MatrixXf> xx_old;
			std::vector<Eigen::MatrixXf> xx_warped;
			std::vector<Eigen::MatrixXf> yy;
			std::vector<Eigen::MatrixXf> yy_inter;
			std::vector<Eigen::MatrixXf> yy_old;
			std::vector<Eigen::MatrixXf> yy_warped;

			/** Matrices that store the depth derivatives */
			Eigen::MatrixXf du;
			Eigen::MatrixXf dv;
			Eigen::MatrixXf dt;

			/** Weights for the range flow constraint equations in the least square solution */
			Eigen::MatrixXf weights;

			/** Matrix which indicates whether the depth of a pixel is zero (null = 1) or not (null = 00).*/
			Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> null;

			/** Least squares covariance matrix */
			Eigen::Matrix<float, 6, 6> est_cov;

			/** Gaussian masks used to build the pyramid and flag to select accurate or fast pyramid*/
			bool fast_pyramid;
			Eigen::Matrix4f f_mask;
			float g_mask[5][5];

			/** Camera properties: */
			float fovh;			//!<Horizontal field of view (rad)
			float fovv;			//!<Vertical field of view (rad)

			/** The maximum resolution that will be considered by the visual odometry method.
			  * As a rule, the higher the resolution the slower but more accurate the method becomes.
			  * They always have to be less or equal to the size of the original depth image. */
			unsigned int rows;
			unsigned int cols;

			/** Aux variables: size from which the pyramid starts to be built */
			unsigned int width;
			unsigned int height;

			/** Aux variables: number of rows and cols at a given level */
			unsigned int rows_i;
			unsigned int cols_i;

			/** Number of coarse-to-fine levels. I has to be consistent with the number of rows and cols, because the
			  * coarsest level cannot be smaller than 15 x 20. */
			unsigned int ctf_levels;

			/** Aux varibles: levels of the image pyramid and the solver, respectively */
			unsigned int image_level;
			unsigned int level;

			/** Speed filter parameters:
			  * Previous_speed_const_weight - Directly weights the previous speed in order to calculate the filtered velocity. Recommended range - (0, 0.5)
			  * Previous_speed_eig_weight - Weights the product of the corresponding eigenvalue and the previous velocity to calculate the filtered velocity*/
			float previous_speed_const_weight;	//!<Default 0.05
			float previous_speed_eig_weight;	//!<Default 0.5

			/** Transformations of the coarse-to-fine levels */
			std::vector<Eigen::MatrixXf> transformations;
			
			/** Solution from the solver at a given level */
			Eigen::Matrix<float, 6, 1> kai_loc_level;

			/** Last filtered velocity in absolute coordinates */
			Eigen::Matrix<float,6,1> kai_abs;

			/** Filtered velocity in local coordinates */
			Eigen::Matrix<float,6,1> kai_loc;
			Eigen::Matrix<float,6,1> kai_loc_old;

			/** Create the gaussian image pyramid according to the number of coarse-to-fine levels */
			void buildCoordinatesPyramid();
			void buildCoordinatesPyramidFast();

			/** Warp the second depth image against the first one according to the 3D transformations accumulated up to a given level */
			void performWarping();

			/** Calculate the "average" coordinates of the points observed by the camera between two consecutive frames and find the Null measurements */
			void calculateCoord();

			/** Calculates the depth derivatives respect to u,v (rows and cols) and t (time) */
			void calculateDepthDerivatives();

			/** This method computes the weighting fuction associated to measurement and linearization errors */
			void computeWeights();

			/** The Solver. It buils the overdetermined system and gets the least-square solution.
			  * It also calculates the least-square covariance matrix */
			void solveOneLevel();

			/** Method to filter the velocity at each level of the pyramid. */
			void filterLevelSolution();

			/** Update camera pose and the velocities for the filter */
			void poseUpdate();


		public:

			/** Frames per second (Hz) */
			float fps;

			/** Resolution of the images taken by the range camera */
			unsigned int cam_mode;	// (1 - 640 x 480, 2 - 320 x 240, 4 - 160 x 120)

			/** Downsample the image taken by the range camera. Useful to reduce the computational burden,
			  * as this downsampling is applied before the gaussian pyramid is built. It must be used when
			    the virtual method "loadFrame()" is implemented */
			unsigned int downsample; // (1 - original size, 2 - res/2, 4 - res/4)

			/** Num of valid points after removing null pixels*/
			unsigned int num_valid_points;

			/** Execution time (ms) */
			float execution_time;

			/** Camera poses */
			mrpt::poses::CPose3D cam_pose;		//!< Last camera pose
			mrpt::poses::CPose3D cam_oldpose;	//!< Previous camera pose

			/** This method performs all the necessary steps to estimate the camera velocity once the new image is read,
			    and updates the camera pose */
			void odometryCalculation();

			/** Get the rows and cols of the depth image that are considered by the visual odometry method. */
			inline void getRowsAndCols(unsigned int &num_rows, unsigned int &num_cols) const {num_rows = rows; num_cols = cols;}

			/** Get the number of coarse-to-fine levels that are considered by the visual odometry method. */
			inline void getCTFLevels(unsigned int &levels) const {levels = ctf_levels;}

			/** Set the horizontal and vertical field of vision (in degrees) */
			inline void setFOV(float new_fovh, float new_fovv);

			/** Get the horizontal and vertical field of vision (in degrees) */
			inline void getFOV(float &cur_fovh, float &cur_fovv) const {cur_fovh = 57.296*fovh; cur_fovv = 57.296*fovv;}

			/** Get the filter constant-weight of the velocity filter. */
			inline float getSpeedFilterConstWeight() const {return previous_speed_const_weight;}

			/** Get the filter eigen-weight of the velocity filter. */
			inline float getSpeedFilterEigWeight() const {return previous_speed_eig_weight;}

			/** Set the filter constant-weight of the velocity filter. */
			inline void setSpeedFilterConstWeight(float new_cweight) { previous_speed_const_weight = new_cweight;}

			/** Set the filter eigen-weight of the velocity filter. */
			inline void setSpeedFilterEigWeight(float new_eweight) { previous_speed_eig_weight = new_eweight;}

			/** Get the coordinates of the points considered by the visual odometry method */
			inline void getPointsCoord(Eigen::MatrixXf &x, Eigen::MatrixXf &y, Eigen::MatrixXf &z);

			/** Get the depth derivatives (last level) respect to u,v and t respectively */
			inline void getDepthDerivatives(Eigen::MatrixXf &cur_du, Eigen::MatrixXf &cur_dv, Eigen::MatrixXf &cur_dt);

			/** Get the camera velocity (vx, vy, vz, wx, wy, wz) expressed in local reference frame estimated by the solver (before filtering) */
			inline mrpt::math::CMatrixFloat61 getSolverSolution() const {return kai_loc_level;}

			/** Get the last camera velocity (vx, vy, vz, wx, wy, wz) expressed in the world reference frame, obtained after filtering */
			inline mrpt::math::CMatrixFloat61 getLastSpeedAbs() const {return kai_abs;}

			/** Get the least-square covariance matrix */
			inline mrpt::math::CMatrixFloat66 getCovariance() const {return est_cov;}

			/** Get the matrix of weights */
			inline void getWeights(Eigen::MatrixXf &we);

			/** Virtual method to be implemented in derived classes.
			  * It should be used to load a new depth image into the variable depth_wf */
			virtual void loadFrame() = 0;

			//Constructor. Initialize variables and matrix sizes
			CDifodo();

		};
	}
}



#endif
