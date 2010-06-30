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

#ifndef mrpt_vision_types_H
#define mrpt_vision_types_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TMatchingPair.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		using namespace mrpt::slam;
		using namespace mrpt::math;
		using namespace mrpt::utils;

		/** Landmark ID */
		typedef	uint64_t TLandmarkID;

		/** Just an index type that uniquely identifies a camera frame. */
		typedef uint64_t TCameraPoseID;

		/** A collection of camera poses from which a given feature has been observed, and its pixel coordinates for each of them. */
		typedef std::map<TCameraPoseID,mrpt::utils::TPixelCoordf> TFeatureObservations;

		/** Input data used by some methods in mrpt::vision     \sa mrpt::vision::camera_calib_ba */
		typedef std::map<TFeatureID, TFeatureObservations>  TSequenceFeatureObservations;

		/** Data returned by  mrpt::vision::camera_calib_ba */
		struct VISION_IMPEXP TCamCalibBAResults
		{
			std::vector<mrpt::poses::CPose3DQuat>  	camera_poses;
			std::vector<mrpt::math::TPoint3D>		landmark_positions;
		};



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

	}
}


#endif
