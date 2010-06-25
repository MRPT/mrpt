/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_vision_utils_H
#define mrpt_vision_utils_H

#include <mrpt/vision/CFeature.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/slam/CObservationVisualLandmarks.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		//class CLandmarksMap;
		//class CObservationVisualLandmarks;
		class CObservationStereoImages;
		class CObservationBearingRange;
	}

	/** Classes for computer vision, detectors, features, etc.
	 */
	namespace vision
	{
		using namespace mrpt::slam;
		using namespace mrpt::math;
		using namespace mrpt::utils;

		// Here follow utility declarations, methods, etc. (Old "VisionUtils" namespace).
		//  Implementations are in "vision.cpp"

		/** Landmark ID
		  */
		typedef	 uint64_t TLandmarkID;

		/** Parameters associated to a stereo system
		  */
		struct VISION_IMPEXP TStereoSystemParams : public mrpt::utils::CLoadableOptions
		{
			/** Initilization of default parameters
			 */
			TStereoSystemParams(	);

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

			/** Method for propagating the feature's image coordinate uncertainty into 3D space. Default value: Prop_Linear
			  */
			enum TUnc_Prop_Method
			{
				/** Linear propagation of the uncertainty
				  */
				Prop_Linear = -1,
				/** Uncertainty propagation through the Unscented Transformation
				  */
				Prop_UT,
				/** Uncertainty propagation through the Scaled Unscented Transformation
				  */
				Prop_SUT
			};

			TUnc_Prop_Method uncPropagation;

			/** Intrinsic parameters
			  */
			CMatrixDouble33	K;
			/** Baseline. Default value: baseline = 0.119f;	[Bumblebee]
			  */
			float		baseline;
			/** Standard deviation of the error in feature detection. Default value: stdPixel = 1
			  */
			float		stdPixel;
			/** Standard deviation of the error in disparity computation. Default value: stdDisp = 1
			  */
			float		stdDisp;
			/** Maximum allowed distance. Default value: maxZ = 20.0f
			  */
			float		maxZ;
			/** Maximum allowed distance. Default value: minZ = 0.5f
			  */
			float		minZ;
			/** Maximum allowed height. Default value: maxY = 3.0f
			  */
			float		maxY;
			/** K factor for the UT. Default value: k = 1.5f
			  */
			float		factor_k;
			/** Alpha factor for SUT. Default value: a = 1e-3
			  */
			float		factor_a;
			/** Beta factor for the SUT. Default value: b = 2.0f
			  */
			float		factor_b;

			/** Parameters initialization
			  */
			//TStereoSystemParams();

		}; // End struct TStereoSystemParams

		/** A structure for storing a 3D ROI
		  */
		struct VISION_IMPEXP TROI
		{
			// Constructors
			TROI();
			TROI(float x1, float x2, float y1, float y2, float z1, float z2);

			// Members
			float	xMin;
			float	xMax;
			float	yMin;
			float	yMax;
			float	zMin;
			float	zMax;
		}; // end struct TROI

		/** A structure for defining a ROI within an image
		  */
		struct VISION_IMPEXP TImageROI
		{
			// Constructors
			TImageROI();
			TImageROI( float x1, float x2, float y1, float y2 );

			// Members
			/** X coordinate limits [0,imageWidth)
			  */
			float	xMin, xMax;
			/** Y coordinate limits [0,imageHeight)
			  */
			float	yMin, yMax;
		}; // end struct TImageROI

		/** A structure containing options for the matching
		  */
		struct VISION_IMPEXP TMatchingOptions : public mrpt::utils::CLoadableOptions
		{

			/** Method for propagating the feature's image coordinate uncertainty into 3D space. Default value: Prop_Linear
			  */
			enum TMatchingMethod
			{
				/** Matching by cross correlation of the image patches
				  */
				mmCorrelation = 0,
				/** Matching by Euclidean distance between SIFT descriptors
				  */
				mmDescriptorSIFT,
				/** Matching by Euclidean distance between SURF descriptors
				  */
				mmDescriptorSURF,
				/** Matching by sum of absolute differences of the image patches
				  */
				mmSAD
			};

			// For determining
			bool	useEpipolarRestriction;		//!< Whether or not take into account the epipolar restriction for finding correspondences
			bool	hasFundamentalMatrix;		//!< Whether or not there is a fundamental matrix
			bool	parallelOpticalAxis;		//!< Whether or not the stereo rig has the optical axes parallel
			bool	useXRestriction;			//!< Whether or not employ the x-coord restriction for finding correspondences (bumblebee camera, for example)

			CMatrixDouble33 F;

			// General
			TMatchingMethod	matching_method;	//!< Matching method
			float	epipolar_TH;				//!< Epipolar constraint (rows of pixels)

			// SIFT
			float	maxEDD_TH;					//!< Maximum Euclidean Distance Between SIFT Descriptors
			float	EDD_RATIO;					//!< Boundary Ratio between the two lowest EDD

			// KLT
			float	minCC_TH;					//!< Minimum Value of the Cross Correlation
			float	minDCC_TH;					//!< Minimum Difference Between the Maximum Cross Correlation Values
			float	rCC_TH;						//!< Maximum Ratio Between the two highest CC values

			// SURF
			float	maxEDSD_TH;					//!< Maximum Euclidean Distance Between SURF Descriptors
			float	EDSD_RATIO;					//!< Boundary Ratio between the two lowest SURF EDSD

			// SAD
			double	minSAD_TH;
			double  SAD_RATIO;

			/** Constructor
			  */
			TMatchingOptions( );

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;

		}; // end struct TMatchingOptions

			/**	Computes the correlation between this image and another one, encapsulating the openCV function cvMatchTemplate
			*   This implementation reduced computation time.
			* \param patch_img The "patch" image, which must be equal, or smaller than "this" image. This function supports gray-scale (1 channel only) images.
			* \param x_search_ini The "x" coordinate of the search window.
			* \param y_search_ini The "y" coordinate of the search window.
			* \param x_search_size The width of the search window.
			* \param y_search_size The height of the search window.
			* \param x_max The x coordinate where found the maximun cross correlation value.
			* \param y_max The y coordinate where found the maximun cross correlation value
			* \param max_val The maximun value of cross correlation which we can find
			*  Note: By default, the search area is the whole (this) image.
			* \sa cross_correlation
			*/
			void VISION_IMPEXP openCV_cross_correlation(
													const CImage	&img,
													const CImage	&patch_img,
													size_t				&x_max,
													size_t				&y_max,
													double				&max_val,
													int					x_search_ini=-1,
													int					y_search_ini=-1,
													int					x_search_size=-1,
													int					y_search_size=-1);

			/**	Invert an image using OpenCV function
			*
			*/
			void VISION_IMPEXP flip(CImage		&img);

			/** Extract a UNITARY 3D vector in the direction of a 3D point, given from its (x,y) pixels coordinates, and the camera intrinsic coordinates.
			  *  \param x Pixels coordinates, from the top-left corner of the image.
			  *  \param y Pixels coordinates, from the top-left corner of the image.
			  *  \param A The 3x3 intrinsic parameters matrix for the camera.
			  *
			  * \sa buildIntrinsicParamsMatrix, defaultIntrinsicParamsMatrix
			  */
			TPoint3D VISION_IMPEXP pixelTo3D( const vision::TPixelCoordf &xy, const CMatrixDouble33 &A);

			/** Builds the intrinsic parameters matrix A from parameters:
			  * \param focalLengthX The focal length, in X (horizontal) pixels
			  * \param focalLengthY The focal length, in Y (vertical) pixels
			  * \param centerX The image center, horizontal, in pixels
			  * \param centerY The image center, vertical, in pixels
			  *
			  * <br>This method returns the matrix:
			  <table>
			  <tr><td>f_x</td><td>0</td><td>cX</td> </tr>
			  <tr><td>0</td><td>f_y</td><td>cY</td> </tr>
			  <tr><td>0</td><td>0</td><td>1</td> </tr>
			  </table>
			  *  See also the tutorial discussing the <a rhref="http://www.mrpt.org/Camera_Parameters">camera model parameters</a>.
			  * \sa defaultIntrinsicParamsMatrix, pixelTo3D
			  */
			CMatrixDouble33 VISION_IMPEXP buildIntrinsicParamsMatrix(
				const double focalLengthX,
				const double focalLengthY,
				const double centerX,
				const double centerY);

			/** Returns the stored, default intrinsic params matrix for a given camera:
			  * \param camIndex  Posible values are listed next.
			  * \param resolutionX The number of pixel columns
			  * \param resolutionY The number of pixel rows
			  *
			  * The matrix is generated for the indicated camera resolution configuration.
			  * The following table summarizes the current supported cameras and the values as
			  *  ratios of the corresponding horz. or vert. resolution:<br>

			  <center><table>
			  <tr>
			   <td><center><b>camIndex</b></center></td>
			   <td><center><b>Manufacturer</b></center></td>
			   <td><center><b>Camera model</b></center></td>
			   <td><center><b>fx</b></center></td>
			   <td><center><b>fy</b></center></td>
			   <td><center><b>cx</b></center></td>
			   <td><center><b>cy</b></center></td>
			  </tr>

			  <tr>
			   <td><center>0</center></td>
			   <td><center>Point Grey Research</center></td>
			   <td><center>Bumblebee</center></td>
			   <td><center>0.79345</center></td>
			   <td><center>1.05793</center></td>
			   <td><center>0.55662</center></td>
			   <td><center>0.52692</center></td>
			  </tr>

			  <tr>
			   <td><center>1</center></td>
			   <td><center>Sony</center></td>
			   <td><center>???</center></td>
			   <td><center>0.95666094</center></td>
			   <td><center>1.3983423f</center></td>
			   <td><center>0.54626328f</center></td>
			   <td><center>0.4939191f</center></td>
			  </tr>
			  </table>
			  </center>

			  * \sa buildIntrinsicParamsMatrix, pixelTo3D
			  */
			CMatrixDouble33 VISION_IMPEXP defaultIntrinsicParamsMatrix(
				unsigned int camIndex = 0,
				unsigned int resolutionX = 320,
				unsigned int resolutionY = 240 );

			/** Explore the feature list and removes features which are in the same coordinates
			  * \param list (Input). The list of features.
			  */
			void VISION_IMPEXP deleteRepeatedFeats( CFeatureList &list );

			/** Search for correspondences which are not in the same row and deletes them
			  * ...
			  */
			void VISION_IMPEXP rowChecking(	CFeatureList &leftList,
								CFeatureList &rightList,
								float threshold = 0.0);

			/** Search for correspondences which are not in the same row and deletes them
			  * ...
			  */
			void VISION_IMPEXP checkTrackedFeatures( CFeatureList &leftList,
							    CFeatureList &rightList,
								vision::TMatchingOptions options);


			/** Computes the dispersion of the features in the image
			  * \param list (IN) Input list of features
			  * \param std	(OUT) 2 element vector containing the standard deviations in the 'x' and 'y' coordinates.
			  * \param mean	(OUT) 2 element vector containing the mean in the 'x' and 'y' coordinates.
			  */
			void VISION_IMPEXP getDispersion( const CFeatureList &list,
											  vector_float &std,
											  vector_float &mean );

			/** Tracks a set of features in an image.
			  */
			void VISION_IMPEXP trackFeatures2( 
				const CImage &inImg1,
				const CImage &inImg2,
				CFeatureList &featureList,
				const unsigned int &window_width = 15,
				const unsigned int &window_height = 15);
			
			/** Tracks a set of features in an image.
			  */
			void VISION_IMPEXP trackFeatures( const CImage &inImg1,
								 const CImage &inImg2,
								 vision::CFeatureList &featureList,
								 const unsigned int &window_width = 15,
								 const unsigned int &window_height = 15 );

			/** Tracks a set of features in an image.
			  */
			void VISION_IMPEXP trackFeatures( const CImage &inImg1,
							 	const CImage &inImg2,
								const CFeatureList &inFeatureList,
								CFeatureList &outFeatureList,
								const unsigned int &window_width,
								const unsigned int &window_height);

			/** Filter bad correspondences by distance
			  * ...
			  */
			void VISION_IMPEXP filterBadCorrsByDistance( mrpt::utils::TMatchingPairList &list,	// The list of correspondences
											unsigned int numberOfSigmas );				// Threshold


			/** Returns a new image where distortion has been removed.
			  * \param A The 3x3 intrinsic parameters matrix
			  * \param dist_coeffs The 1x4 (or 1x5) vector of distortion coefficients
			  */
			inline void correctDistortion(
					const CImage	&in_img,
					CImage			&out_img,
					const CMatrixDouble33	&A,
					const vector_double &dist_coeffs )
			{
				in_img.rectifyImage( out_img, A, dist_coeffs);
			}


			/** Computes the mean squared distance between a set of 3D correspondences
			  * ...
			  */
			double VISION_IMPEXP computeMsd( const mrpt::utils::TMatchingPairList &list,
								const mrpt::poses::CPose3D &Rt );

			/** Transform two clouds of 3D points into a matched list of points
			  * ...
			  */
			void VISION_IMPEXP cloudsToMatchedList( const mrpt::slam::CObservationVisualLandmarks &cloud1,
									   const mrpt::slam::CObservationVisualLandmarks &cloud2,
											 mrpt::utils::TMatchingPairList &outList);

			/** Computes the main orientation of a set of points with an image (for using in SIFT-based algorithms)
			  * \param image (Input). The input image.
			  * \param x (Input). A vector containing the 'x' coordinates of the image points.
			  * \param y (Input). A vector containing the 'y' coordinates of the image points.
			  * \param orientation (Output). A vector containing the main orientation of the image points.
			  */
			float VISION_IMPEXP computeMainOrientation( const CImage &image,
										  const unsigned int &x,
										  const unsigned int &y );

			/** Find the matches between two lists of features. They must be of the same type. Return value: the number of matched pairs of features
			  * \param list1 (Input). One list.
			  * \param list2 (Input). Other list.
			  * \param matches (Output). A vector of pairs of correspondences.
			  * \param options (Optional Input). A struct containing matching options
			  */
			size_t VISION_IMPEXP matchFeatures( const CFeatureList &list1,
								  const CFeatureList &list2,
								  CMatchedFeatureList &matches,
								  const TMatchingOptions &options = TMatchingOptions() );

			/** Find the matches between two lists of features. They must be of the same type. Return value: the number of matched pairs of features
			  * \param list1 (Input). One list.
			  * \param list2 (Input). Other list.
			  * \param matches (Output). A vector of pairs of correspondences.
			  * \param options (Optional Input). A struct containing matching options
			  */
			size_t VISION_IMPEXP matchFeatures2( const CFeatureList &list1,
								  const CFeatureList &list2,
								  CMatchedFeatureList &matches,
								  const TMatchingOptions &options = TMatchingOptions() );

			void VISION_IMPEXP addFeaturesToImage( 
				const CImage &inImg, 
				const CFeatureList &theList, 
				CImage &outImg );

			/** Project a list of matched features into the 3D space, using the provided options for the stereo system
			  * \param matches (Input). The list of matched features.
			  * \param options (Input). The options of the stereo system.
			  * \param landmarks (Output). A map containing the projected landmarks.
			  */
			void VISION_IMPEXP projectMatchedFeatures(
				CMatchedFeatureList					&mfList,		// The set of matched features
				const vision::TStereoSystemParams	&param,			// Parameters for the stereo system
				mrpt::slam::CLandmarksMap			&landmarks );	// Output map of 3D landmarks

			/** Project a pair of feature lists into the 3D space, using the provided options for the stereo system. The matches must be in order, 
			  *	i.e. leftList[0] corresponds to rightList[0] and so on;
			  * \param leftList (Input). The left list of matched features.
			  * \param rightList (Input). The right list of matched features.
			  * \param options (Input). The options of the stereo system.
			  * \param landmarks (Output). A map containing the projected landmarks.
			  */
			void VISION_IMPEXP projectMatchedFeatures(
				CFeatureList						&leftList,		// The left of matched features (matches must be ordered!)
				CFeatureList						&rightList,		// The right of matched features (matches must be ordered!)
				const vision::TStereoSystemParams	&param,			// Parameters for the stereo system
				mrpt::slam::CLandmarksMap			&landmarks );	// Output map of 3D landmarks

			/** Data associated to each image in the calibration process mrpt::vision::checkerBoardCameraCalibration (All the information can be left empty and will be filled up in the calibration method).
			  */
			struct TImageCalibData
			{
				CImage	img_original;     //!< This image will be automatically loaded from the file name passed to checkerBoardCameraCalibration
				CImage	img_checkboard;   //!< At output, this will contain the detected checkerboard overprinted to the image.
				CImage	img_rectified;    //!< At output, this will be the rectified image
				std::vector<mrpt::poses::CPoint2D>	detected_corners; //!< At output, the detected corners (x,y) in pixel units.
				mrpt::poses::CPose3D			reconstructed_camera_pose;   //!< At output, the reconstructed pose of the camera.
				std::vector<TPixelCoordf>		projectedPoints_distorted;   //!< At output, only will have an empty vector if the checkerboard was not found in this image, or the predicted (reprojected) corners, which were used to estimate the average square error.
				std::vector<TPixelCoordf>		projectedPoints_undistorted; //!< At output, like projectedPoints_distorted but for the undistorted image.
			};

			/**  A list of images, used in checkerBoardCameraCalibration
			  * \sa checkerBoardCameraCalibration
			  */
			typedef std::map<std::string,TImageCalibData> TCalibrationImageList;

			/** Performs a camera calibration (computation of projection and distortion parameters) from a sequence of captured images of a checkerboard.
			  * \param input_images [IN/OUT] At input, this list must have one entry for each image to process. At output the original, detected checkboard and rectified images can be found here. See TImageCalibData.
			  * \param check_size_x [IN] The number of squares in the checkerboard in the X direction.
			  * \param check_size_y [IN] The number of squares in the checkerboard in the Y direction.
			  * \param check_squares_length_X_meters [IN] The size of each square in the checkerboard, in meters, in the X axis.
			  * \param check_squares_length_Y_meters [IN] This will typically be equal to check_squares_length_X_meters.
			  * \param intrinsicParams [OUT] The 3x3 intrinsic parameters matrix. See http://www.mrpt.org/Camera_Parameters
			  * \param distortionParams [OUT] The 1x4 vector of distortion parameters: k1 k2 p1 p2. See http://www.mrpt.org/Camera_Parameters
			  * \param normalize_image [IN] Select OpenCV flag
			  * \param out_MSE  [OUT] If set to !=NULL, the mean square error of the reprojection will be stored here (in pixel units).
			  * \param skipDrawDetectedImgs [IN] Whether to skip the generation of the undistorted and detected images in each TImageCalibData
			  * \sa The <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui application</a> is a user-friendly GUI to this class.
			  * \return false on any error (more info will be dumped to cout), or true on success.
			  * \sa CImage::findChessboardCorners
			  */
			bool VISION_IMPEXP checkerBoardCameraCalibration(
				TCalibrationImageList &images,
				unsigned int  check_size_x,
				unsigned int  check_size_y,
				double        check_squares_length_X_meters,
				double        check_squares_length_Y_meters,
				CMatrixDouble33			&intrinsicParams,
				std::vector<double>		&distortionParams,
				bool		normalize_image = true,
				double            *out_MSE = NULL,
				bool               skipDrawDetectedImgs = false
				);

			/** Converts a stereo images observation into a bearing and range observation.
				\param inObs	[IN]	The input stereo images observation.
				\param sg		[IN]	The sigma of the row, col, and disparity variables involved in the feature detection.
				\param outObs	[OUT]	The output bearing and range observation (including covariances).
			*/
			void VISION_IMPEXP StereoObs2BRObs(
				const CObservationStereoImages &inObs,
				const std::vector<double> &sg,
				CObservationBearingRange &outObs
				);

			/** Converts a matched feature list into a bearing and range observation (some of the stereo camera system must be provided).
				\param inMatches		[IN]	The input list of matched features.
				\param intrinsicParams	[IN]	The intrisic params of the reference (left) camera of the stereo system.
				\param baseline			[IN]	The distance among the X axis of the right camera wrt the reference (left) camera.
				\param sg				[IN]	The sigma of the row, col, and disparity variables involved in the feature detection.
				\param outObs			[OUT]	The output bearing and range observation (including covariances).
			*/
			void VISION_IMPEXP StereoObs2BRObs(
				const CMatchedFeatureList &inMatches,
				const CMatrixDouble33 &intrinsicParams,
				const double &baseline,
				const CPose3D &sensorPose,
				const std::vector<double> &sg,
				CObservationBearingRange &outObs
				);
			
			/** Converts a CObservationVisualLandmarks into a bearing and range observation (without any covariances). Fields of view are not computed.
				\param inObs			[IN]	The input observation.
				\param outObs			[OUT]	The output bearing and range observation.
			*/
			void VISION_IMPEXP StereoObs2BRObs( 
				const CObservationVisualLandmarks &inObs, 
				CObservationBearingRange &outObs 
				);
	}
}


#endif
