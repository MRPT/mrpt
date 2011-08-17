/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/vision/types.h>
#include <mrpt/vision/chessboard_camera_calib.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		class CObservationStereoImages;
		class CObservationBearingRange;
		class CLandmarksMap;
		class CObservationVisualLandmarks;
	}

	/** Classes for computer vision, detectors, features, etc.  \ingroup mrpt_vision_grp
	 */
	namespace vision
	{
		using std::vector;
		using namespace mrpt::slam;
		using namespace mrpt::math;
		using namespace mrpt::utils;

		/** \addtogroup mrpt_vision_grp
		  *  @{ */

			/**	Computes the correlation between this image and another one, encapsulating the openCV function cvMatchTemplate
			*   This implementation reduced computation time.
			* \param img            [IN]    The imput image. This function supports gray-scale (1 channel only) images.
			* \param patch_img      [IN]    The "patch" image, which must be equal, or smaller than "this" image. This function supports gray-scale (1 channel only) images.
			* \param x_max          [OUT]   The x coordinate where it was found the maximun cross correlation value.
			* \param y_max          [OUT]   The y coordinate where it was found the maximun cross correlation value.
			* \param max_val        [OUT]   The maximun value of cross correlation which we can find
			* \param x_search_ini   [IN]    The "x" coordinate of the search window.
			* \param y_search_ini   [IN]    The "y" coordinate of the search window.
			* \param x_search_size  [IN]    The width of the search window.
			* \param y_search_size  [IN]    The height of the search window.
			*  Note: By default, the search area is the whole (this) image.
			* \sa cross_correlation
			*/
			void VISION_IMPEXP openCV_cross_correlation(
                                const CImage	        & img,
								const CImage	        & patch_img,
								size_t				    & x_max,
								size_t				    & y_max,
								double				    & max_val,
								int					    x_search_ini=-1,
								int					    y_search_ini=-1,
								int					    x_search_size=-1,
								int					    y_search_size=-1);

			/**	Invert an image using OpenCV function
			*
			*/
			void VISION_IMPEXP flip(
                                CImage		            & img);

			/** Extract a UNITARY 3D vector in the direction of a 3D point, given from its (x,y) pixels coordinates, and the camera intrinsic coordinates.
			  *  \param xy  [IN]   Pixels coordinates, from the top-left corner of the image.
			  *  \param A   [IN]   The 3x3 intrinsic parameters matrix for the camera.
			  *  \return The TPoint3D containing the output unitary vector.
			  * \sa buildIntrinsicParamsMatrix, defaultIntrinsicParamsMatrix, TPixelCoordf
			  */
			TPoint3D VISION_IMPEXP pixelTo3D(
                                const TPixelCoordf      & xy,
                                const CMatrixDouble33   & A);

			/** Builds the intrinsic parameters matrix A from parameters:
			  * \param focalLengthX [IN]   The focal length, in X (horizontal) pixels
			  * \param focalLengthY [IN]   The focal length, in Y (vertical) pixels
			  * \param centerX      [IN]   The image center, horizontal, in pixels
			  * \param centerY      [IN]   The image center, vertical, in pixels
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
                                const double            focalLengthX,
                                const double            focalLengthY,
                                const double            centerX,
                                const double            centerY);

			/** Returns the stored, default intrinsic params matrix for a given camera:
			  * \param camIndex     [IN]   Posible values are listed next.
			  * \param resolutionX  [IN]   The number of pixel columns
			  * \param resolutionY  [IN]   The number of pixel rows
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
                                unsigned int            camIndex = 0,
                                unsigned int            resolutionX = 320,
                                unsigned int            resolutionY = 240 );

			/** Explore the feature list and removes features which are in the same coordinates
			  * \param list [IN] The list of features.
			  */
			void VISION_IMPEXP deleteRepeatedFeats(
                                CFeatureList            & list );

			/** Search for correspondences which are not in the same row and deletes them
			  * \param leftList     [IN/OUT]    The left list of matched features.
			  * \param rightList    [IN/OUT]    The right list of matched features.
			  * \param threshold    [IN]        The tolerance value for the row checking: valid matched are within this threshold.
			  */
			void VISION_IMPEXP rowChecking(
                                CFeatureList            & leftList,
								CFeatureList            & rightList,
								float                   threshold = 1.0);

			/** Computes the dispersion of the features in the image
			  * \param list [IN]    Input list of features
			  * \param std	[OUT]   2 element vector containing the standard deviations in the 'x' and 'y' coordinates.
			  * \param mean	[OUT]   2 element vector containing the mean in the 'x' and 'y' coordinates.
			  */
			void VISION_IMPEXP getDispersion(
                                const CFeatureList      & list,
								vector_float            & std,
								vector_float            & mean );

			/** Returns a new image where distortion has been removed.
			  * \param A The 3x3 intrinsic parameters matrix
			  * \param dist_coeffs The 1x4 (or 1x5) vector of distortion coefficients
			  */
			inline void correctDistortion(
                                const CImage	        & in_img,
                                CImage			        & out_img,
                                const CMatrixDouble33	& A,
                                const vector_double     & dist_coeffs )
			{
				in_img.rectifyImage( out_img, A, dist_coeffs);
			}


			/** Computes the mean squared distance between a set of 3D correspondences
			  * ...
			  */
			double VISION_IMPEXP computeMsd(
                                const TMatchingPairList & list,
								const poses::CPose3D    & Rt );

			/** Transform two clouds of 3D points into a matched list of points
			  * ...
			  */
			void VISION_IMPEXP cloudsToMatchedList(
                                const CObservationVisualLandmarks   & cloud1,
                                const CObservationVisualLandmarks   & cloud2,
								TMatchingPairList                   & outList);

			/** Computes the main orientation of a set of points with an image (for using in SIFT-based algorithms)
			  * \param image    [IN] The input image.
			  * \param x        [IN] A vector containing the 'x' coordinates of the image points.
			  * \param y        [IN] A vector containing the 'y' coordinates of the image points.
			  * \return The main orientation of the image point.
			  */
			float VISION_IMPEXP computeMainOrientation(
                                const CImage                        & image,
								unsigned int                        x,
								unsigned int                        y );

			/** Normalizes the brigthness and contrast of an image by setting its mean value to zero and its standard deviation to unit.
			  * \param image        [IN]        The input image.
			  * \param nimage       [OUTPUT]    The new normalized image.
			  */
            void VISION_IMPEXP normalizeImage(
                                const CImage                        & image,
                                CImage                              & nimage );

			/** Find the matches between two lists of features which must be of the same type.
			  * \param list1    [IN]    One list.
			  * \param list2    [IN]    Other list.
			  * \param matches  [OUT]   A vector of pairs of correspondences.
			  * \param options  [IN]    A struct containing matching options
			  * \return Returns the number of matched pairs of features.
			  */
			size_t VISION_IMPEXP matchFeatures(
                                const CFeatureList                  & list1,
                                const CFeatureList                  & list2,
                                CMatchedFeatureList                 & matches,
                                const TMatchingOptions              & options = TMatchingOptions(),
                                const TStereoSystemParams           & params = TStereoSystemParams() );

            /** Calculates the Sum of Absolutes Differences (range [0,1]) between two patches. Both patches must have the same size.
			  * \param mList    [IN]  The list of matched features.
			  * \param mask1    [OUT] The output mask for left features.
			  * \param mask2    [OUT] The output mask for right features.
			  * \param wSize    [IN] The value of the masking window for each features.
			  * \exception if mList.size() = 0
	          */
            void VISION_IMPEXP generateMask(
                                const CMatchedFeatureList           & mList,
                                CMatrixBool                         & mask1,
                                CMatrixBool                         & mask2,
                                int                                 wSize = 10 );

            /** Calculates the Sum of Absolutes Differences (range [0,1]) between two patches. Both patches must have the same size.
			  * \param patch1 [IN]  One patch.
			  * \param patch2 [IN]  The other patch.
			  * \return The value of computed SAD normalized to [0,1]
	          */
	        double VISION_IMPEXP computeSAD(
                                const CImage                        & patch1,
                                const CImage                        & patch2 );

			/** Draw rectangles around each of the features on a copy of the input image.
			  * \param inImg    [IN]    The input image where to draw the features.
			  * \param theList  [IN]    The list of features.
			  * \param outImg   [OUT]   The copy of the input image with the marked features.
			  */
			void VISION_IMPEXP addFeaturesToImage(
                                const CImage                        & inImg,
                                const CFeatureList                  & theList,
                                CImage                              & outImg );

			/** Computes the 3D position of a set of matched features from their coordinates in the images. The list have to be matched in order, e.g. leftList[0]<->rightList[0]
			  * \param leftList     [IN]    The left list of features.
			  * \param rightList    [IN]    The right list of features.
			  * \param vP3D         [OUT]   A vector of TPoint3D containing the 3D positions of the projected points.
			  * \param params       [IN]    The intrinsic and extrinsic parameters of the stereo pair.
			  */
            void VISION_IMPEXP projectMatchedFeatures(
                                const CFeatureList			        & leftList,
                                const CFeatureList			        & rightList,
                                vector<TPoint3D>                    & vP3D,
                                const TStereoSystemParams           & params = TStereoSystemParams() );

			/** Computes the 3D position of a particular matched feature.
			  * \param leftList     [IN]    The left feature.
			  * \param rightList    [IN]    The right feature.
			  * \param vP3D         [OUT]   The 3D position of the projected point.
			  * \param params       [IN]    The intrinsic and extrinsic parameters of the stereo pair.
			  */
            void VISION_IMPEXP projectMatchedFeature(
                                const CFeaturePtr                   & leftFeat,
                                const CFeaturePtr                   & rightFeat,
                                TPoint3D                            & p3D,
                                const TStereoSystemParams           & params = TStereoSystemParams() );

			/** Project a list of matched features into the 3D space, using the provided parameters of the stereo system
			  * \param mfList       [IN/OUT]    The list of matched features. Features which yields a 3D point outside the area defined in TStereoSystemParams are removed from the lists.
			  * \param param        [IN]        The parameters of the stereo system.
			  * \param landmarks    [OUT]       A map containing the projected landmarks.
			  * \sa TStereoSystemParams, CLandmarksMap
			  */
			void VISION_IMPEXP projectMatchedFeatures(
                                CMatchedFeatureList			        & mfList,
                                const TStereoSystemParams	        & param,
                                CLandmarksMap			            & landmarks );

			/** Project a pair of feature lists into the 3D space, using the provided options for the stereo system. The matches must be in order,
			  *	i.e. leftList[0] corresponds to rightList[0] and so on. Features which yields a 3D point outside the area defined in TStereoSystemParams are removed from the lists.
			  * \param leftList     [IN/OUT]    The left list of matched features.
			  * \param rightList    [IN/OUT]    The right list of matched features.
			  * \param param        [IN]        The options of the stereo system.
			  * \param landmarks    (OUT]       A map containing the projected landmarks.
			  * \sa TStereoSystemParams, CLandmarksMap
			  */
			void VISION_IMPEXP projectMatchedFeatures(
                                CFeatureList				        & leftList,
                                CFeatureList					    & rightList,
                                const TStereoSystemParams	        & param,
                                CLandmarksMap			            & landmarks );

			/** Converts a stereo images observation into a bearing and range observation.
				\param inObs	[IN]	The input stereo images observation.
				\param sg		[IN]	The sigma of the row, col, and disparity variables involved in the feature detection.
				\param outObs	[OUT]	The output bearing and range observation (including covariances).
			*/
			void VISION_IMPEXP StereoObs2BRObs(
                                const CObservationStereoImages      & inObs,
                                const vector<double>                & sg,
                                CObservationBearingRange            & outObs );

			/** Converts a matched feature list into a bearing and range observation (some of the stereo camera system must be provided).
				\param inMatches		[IN]	The input list of matched features.
				\param intrinsicParams	[IN]	The intrisic params of the reference (left) camera of the stereo system.
				\param baseline			[IN]	The distance among the X axis of the right camera wrt the reference (left) camera.
				\param sg				[IN]	The sigma of the row, col, and disparity variables involved in the feature detection.
				\param outObs			[OUT]	The output bearing and range observation (including covariances).
			*/
			void VISION_IMPEXP StereoObs2BRObs(
                                const CMatchedFeatureList           & inMatches,
                                const CMatrixDouble33               & intrinsicParams,
                                const double                        & baseline,
                                const CPose3D                       & sensorPose,
                                const vector<double>                & sg,
                                CObservationBearingRange            & outObs );

			/** Converts a CObservationVisualLandmarks into a bearing and range observation (without any covariances). Fields of view are not computed.
				\param inObs			[IN]	The input observation.
				\param sg				[IN]	The sigma of the row, col, and disparity variables involved in the feature detection.
				\param outObs			[OUT]	The output bearing and range observation.
			*/
            void VISION_IMPEXP StereoObs2BRObs(
                                const CObservationStereoImages      & inObs,
                                const vector<double>                & sg,
                                CObservationBearingRange            & outObs );

			/** Converts a CObservationVisualLandmarks into a bearing and range observation (without any covariances). Fields of view are not computed.
				\param inObs			[IN]	The input observation.
				\param outObs			[OUT]	The output bearing and range observation.
			*/
			void VISION_IMPEXP StereoObs2BRObs(
                                const CObservationVisualLandmarks   & inObs,
                                CObservationBearingRange            & outObs );

            /** Computes a pair of x-and-y maps for stereo rectification from a pair of cameras and the relative pose of the second one wrt the first one.
                \param cam1, cam2           [IN]    The pair of involved cameras
                \param rightCameraPose      [IN]    The change in pose of the second camera wrt the first one
                \param outMap1x,outMap1y    [OUT]   The x-and-y maps corresponding to cam1 (should be converted to *cv::Mat)
                \param outMap2x,outMap2y    [OUT]   The x-and-y maps corresponding to cam2 (should be converted to *cv::Mat)
            */
            void VISION_IMPEXP computeStereoRectificationMaps(
                                const TCamera                       & cam1,
                                const TCamera                       & cam2,
                                const poses::CPose3D                & rightCameraPose,
                                void                                *outMap1x,
                                void                                *outMap1y,
                                void                                *outMap2x,
                                void                                *outMap2y );


	/** @} */ // end of grouping
		
	} // end-namespace-vision
} // end-namespace-mrpt


#endif
