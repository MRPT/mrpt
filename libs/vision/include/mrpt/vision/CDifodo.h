/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDifodo_H
#define CDifodo_H

#include <mrpt/base.h>
#include <Eigen/Dense>
#include <mrpt/vision/link_pragmas.h>


//class CDeflodo Acronim for "Depth Flow odometry" -> otra opción de nombre...
namespace mrpt
{
	namespace vision
	{
		using mrpt::poses::CPose3D;
		using Eigen::MatrixXf;
		using Eigen::MatrixXi;

		/** This abstract class implements a method called "Difodo" to perform Visual odometry with range cameras.
		*	It is based on the range flow equation and assumes that the scene is rigid.
		*	It can work with different image resolutions (640 x 480, 320 x 240 or 160 x 120).
		*	Independently of the initial resolution chosen, the method normally makes use of a smaller amount of points
		*	which can be adjusted with the member variables (rows,cols).
		*
		*   How to use:
		*		- A derived class must be created which defines the method "loadFrame(...)" according to the user application.
		*		  This method has to load the depth image into the variable "depth_wf".
		*		- Call loadFrame();
		*		- Call OdometryCalculation();
		*		- Call filterSpeedAndPoseUpdate();
		*
		*	For further information have a look at the apps:
		*    - [Difodometry-Camera](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-camera/)
		*    - [Difodometry-Datasets](http://www.mrpt.org/list-of-mrpt-apps/application-difodometry-datasets/)
		*
		*	Please refer to the respective publication when using this method: *************************
		*
		* - JUN/2013: First design.
		* - JAN/2014: Integrated into MRPT library.
		*
		*  \sa *********************
		*  \ingroup mrpt_vision_grp
		*/

		class VISION_IMPEXP CDifodo {

		protected:

			/** Matrices that store the original and filtered depth frames with the image resolution */
			MatrixXf depth_ft;
			MatrixXf depth_wf;

			/** Matrices that store the point coordinates after downsampling. */
			MatrixXf depth;
			MatrixXf depth_old;
			MatrixXf depth_inter;
			MatrixXf xx;
			MatrixXf xx_inter;
			MatrixXf xx_old;
			MatrixXf yy;
			MatrixXf yy_inter;
			MatrixXf yy_old;

			/** Matrices that store the depth derivatives */
			MatrixXf du;
			MatrixXf dv;
			MatrixXf dt;

			/** Matrix which indicates wheter the depth of a pixel is zero (null = 1) or not (null = 00). and border and noisy points */
			MatrixXi null;

			/** Matrix which indicates wheter a point is in a border or has an inaccurate depth (border =1, border = 0 otherwise) */
			MatrixXi border;

			/** Least squares covariance matrix */
			math::CMatrixFloat66 est_cov;

			/** Camera properties: */
			float f_dist;		//!<Focal lenght (meters)
			float x_incr;		//!<Separation between pixels (cols) in the sensor array (meters)
			float y_incr;		//!<Separation between pixels (rows) in the sensor array (meters)
			float fovh;			//!<Horizontal field of view (rad)
			float fovv;			//!<Vertical field of view (rad)

			/** Number of rows and cols of the depth image that will be considered by the visual odometry method.
			  * As a rule, the more rows and cols the slower and more accurate the method becomes.
			  * They always have to be less or equal to the size of the original depth image. */
			unsigned int rows;
			unsigned int cols;

			/** Size (rows) of the gaussian kernel used to filter the depth image */
			unsigned int gaussian_mask_size;

			/** Speed filter parameters:
			  * Previous_speed_const_weight directly ponders the previous speed in order to calculate the filtered speed. Recommended range - (0, 0.5)
			  * Previous_speed_eig_weight ponders the product of the corresponding eigenvalue and the previous speed in order to calculate the filtered speed*/
			float previous_speed_const_weight;	//!<Default 0.2
			float previous_speed_eig_weight;	//!<Default 400

			/** Solution from the solver: kai before applying the filter in local coordinates */
			math::CMatrixFloat61 kai_solver;

			/** Last filtered speed in absolute coordinates */
			math::CMatrixFloat61 kai_abs;

			/** It filters the depth image with a gaussian kernel and downsample this filtered image according to the values of "rows" and "cols" */
			void filterAndDownsample();

			/** It calculates the "average" coordinates of the points observed by the camera between two consecutive frames */
			void calculateCoord();

			/** It calculates the depth derivatives respect to u,v (rows and cols) and t (time) */
			void calculateDepthDerivatives();

			/** This method finds pixels whose depth is zero to subsequently discard them */
			void findNullPoints();

			/** This method finds pixels which are not in planar or smooth surfaces, and also inaccurate (noisy) pixels */
			void findBorders();

			/** This method discards the pixels found by 'findNullPoints()' and 'findBorders()' */
			void findValidPoints();

			/** The Solver. It buils the overdetermined system and gets the least-square solution.
			  * It also calculates the least-square covariance matrix */
			void solveDepthSystem();

			/**  Virtual method to filter the speed and update the camera pose. */
			virtual void filterSpeedAndPoseUpdate();

		public:

			/** Camera properties */
			float lens_disp;	//Lateral displacement of the lens with respect to the center of the camera (meters)

			/** Frames per second (Hz) */
			float fps;

			/** Resolution of the images taken by the range camera */
			unsigned int cam_mode;	// (1 - 640 x 480, 2 - 320 x 240, 4 - 160 x 120)

			/** Downsample the image taken by the range camera. Useful to reduce the computational burden,
			  * as this downsampling is applied before the gaussian filter */
			unsigned int downsample; // (1 - original size, 2 - res/2, 4 - res/4)

			/** Num of valid points after removing null pixels and borders */
			unsigned int num_valid_points;

			/** Thresholds used to remove borders and noisy points */
			float	duv_threshold;		//!< Threshold to du*du + dv*dv
			float	dt_threshold;		//!< Threshold to dt
			float	dif_threshold;		//!< Threshold to [abs(final_dx-ini_dx) + abs(final_dy-ini_dy)]
			float	difuv_surroundings;	//!< Threshold to the difference of (du,dv) at a pixel and the values of (du,dv) at its surroundings
			float	dift_surroundings;	//!< Threshold to the difference of dt at a pixel and the value of dt at its surroundings

			/** Execution time (ms) */
			float execution_time;

			/** Camera poses */
			CPose3D cam_pose;		//!< Last camera pose
			CPose3D cam_oldpose;	//!< Previous camera pose

			/** This method performs the necessary steps to estimate the camera speed in local coordinates once the depth image has been loaded */
			void OdometryCalculation();

			/** It sets the camera focal lenght */
			inline void setCameraFocalLenght(float new_f);

			/** It gets the camera focal lenght */
			inline float getCameraFocalLenght() const {return f_dist;}

			/** It sets the rows and cols of the depth image that will be considered by the visual odometry method. */
			inline void setRowsAndCols(unsigned int num_rows, unsigned int num_cols);

			/** It gets the rows and cols of the depth image that are considered by the visual odometry method. */
			inline void getRowsAndCols(unsigned int &num_rows, unsigned int &num_cols) const {num_rows = rows; num_cols = cols;}

			/** It sets the horizontal and vertical field of vision (in degrees) */
			inline void setFOV(float new_fovh, float new_fovv);

			/** It gets the horizontal and vertical field of vision (in degrees) */
			inline void getFOV(float &cur_fovh, float &cur_fovv) const {cur_fovh = 57.296*fovh; cur_fovv = 57.296*fovv;}

			/** It gets the weight that ponders the previous speed in order to calculate the filtered speed. */
			inline float getSpeedFilterConstWeight() const {return previous_speed_const_weight;}

			/** It gets the weight that ponders the product of the corresponding eigenvalue and the previous speed in order to calculate the filtered speed */
			inline float getSpeedFilterEigWeight() const {return previous_speed_eig_weight;}

			/** It sets the weight that ponders the previous speed in order to calculate the filtered speed. */
			inline void setSpeedFilterConstWeight(float new_cweight) { previous_speed_const_weight = new_cweight;}

			/** It sets the weight that ponders the product of the corresponding eigenvalue and the previous speed in order to calculate the filtered speed */
			inline void setSpeedFilterEigWeight(float new_eweight) { previous_speed_eig_weight = new_eweight;}

			/** This method gets the coordinates of the points regarded by the visual odometry method */
			inline void getPointsCoord(MatrixXf &x, MatrixXf &y, MatrixXf &z);

			/** This method gets the depth derivatives respect to u,v and t respectively */
			inline void getDepthDerivatives(MatrixXf &cur_du, MatrixXf &cur_dv, MatrixXf &cur_dt);

			/** It gets the camera speed (vx, vy, vz, wx, wy, wz) expressed in local reference frame estimated by the solver (before filtering) */
			inline mrpt::math::CMatrixFloat61 getSolverSolution() const {return kai_solver;}

			/** It gets the last camera speed (vx, vy, vz, wx, wy, wz) expressed in the world reference frame, obtained after filtering */
			inline mrpt::math::CMatrixFloat61 getLastSpeedAbs() const {return kai_abs;}

			/** It gets the least-square covariance matrix */
			inline mrpt::math::CMatrixFloat66 getCovariance() const {return est_cov;}

			/** It resets the border thresholds to their default values */
			void bordersThresholdToDefault();

			/** Virtual method to be implemented in derived classes.
			  * It should be used to load the last depth image into the variable depth_wf */
			virtual void loadFrame() = 0;

			//Constructor. Initialize variables and matrix sizes
			CDifodo();

		};
	}
}



#endif
