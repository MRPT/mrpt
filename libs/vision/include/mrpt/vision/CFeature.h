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
#ifndef CFeature_H
#define CFeature_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/stl_extensions.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/ops_matrices.h>

#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		using namespace mrpt::utils;
		using namespace mrpt::math;

		class CFeatureList;
		class CMatchedFeatureList;

		/** Definition of a feature ID
		*/
		typedef uint64_t TFeatureID;

		/** Types of features - This means that the point has been detected with this algorithm, which is independent of additional descriptors a feature may also have
		*/
		enum TFeatureType
		{
			featNotDefined = -1,	//!< Non-defined feature (also used for Occupancy features)
			featKLT = 0,			//!< Kanade-Lucas-Tomasi feature [SHI'94]
			featHarris,				//!< Harris border and corner detector [HARRIS]
			featBCD,				//!< Binary corder detector
			featSIFT,				//!< Scale Invariant Feature Transform [LOWE'04]
			featSURF,				//!< Speeded Up Robust Feature [BAY'06]
			featBeacon,				//!< A especial case: this is not an image feature, but a 2D/3D beacon (used for range-only SLAM from mrpt::slam::CLandmark)
			featFAST				//!< FAST feature detector ("Faster and better: A machine learning approach to corner detection", E. Rosten, R. Porter and T. Drummond, PAMI, 2009).
		};

		/** The bitwise OR combination of values of TDescriptorType are used in CFeatureExtraction::computeDescriptors to indicate which descriptors are to be computed for features.
		  */
		enum TDescriptorType
		{
			descAny				= 0,  //!< Used in some methods to mean "any of the present descriptors"
			descSIFT            = 1,  //!< SIFT descriptors
			descSURF			= 2,  //!< SURF descriptors
			descSpinImages      = 4,  //!< Intensity-domain spin image descriptors
			descPolarImages     = 8,  //!< Polar image descriptor
			descLogPolarImages	= 16  //!< Log-Polar image descriptor
		};

		enum TKLTFeatureStatus
		{
			/** Inactive
			*/
			statusKLT_IDLE,
			/** Out Of Bounds
			*/
			statusKLT_OOB,
			/** Determinant of the matrix too small
			*/
			statusKLT_SMALL_DET,
			/** Error too big
			*/
			statusKLT_LARGE_RESIDUE,
			/**
			*/
			statusKLT_MAX_RESIDUE,
			/** Feature correctly tracked
			*/
			statusKLT_TRACKED,
			/** Iteration maximum reached
			*/
			statusKLT_MAX_ITERATIONS
		};

		/****************************************************
						Class CFEATURE
		*****************************************************/
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CFeature, mrpt::utils::CSerializable, VISION_IMPEXP )

		/** A generic 2D feature from an image, extracted with \a CFeatureExtraction
		  * Each feature may have one or more descriptors (see \a descriptors), in addition to an image patch.
		  * The (Euclidean) distance between descriptors in a pair of features can be computed with  descriptorDistanceTo,
		  *  while the similarity of the patches is given by patchCorrelationTo.
		  */
		class VISION_IMPEXP CFeature : public mrpt::utils::CSerializable
		{
			friend class CFeatureList;
			friend class CMatchedFeatureList;

			DEFINE_SERIALIZABLE( CFeature )

		public:
			float				x,y;			//!< Coordinates in the image
			TFeatureID			ID;				//!< ID of the feature
			CImage				patch;			//!< A patch of the image surrounding the feature
			uint16_t			patchSize;		//!< Size of the patch (patchSize x patchSize) (it must be an odd number)
			TFeatureType		type;			//!< Type of the feature: featNotDefined, featSIFT, featKLT,	featHarris, featSURF, featBeacon
			TKLTFeatureStatus	KLT_status;		//!< Status of the feature tracking process
			float				KLT_val;		//!< Value of the goodness of the feature
			float				orientation;	//!< Main orientation of the feature
			float				scale;			//!< Feature scale into the scale space
			uint8_t				IDSourceImage;	//!< ID of the image from which the feature was extracted.

			bool isPointFeature() const;		//!< Return false only for Blob detectors (SIFT, SURF)

			/** All the possible descriptors this feature may have */
			struct VISION_IMPEXP TDescriptors
			{
				TDescriptors();  // Initialization

				std::vector<unsigned char>	SIFT;			//!< Feature descriptor
				std::vector<float>			SURF;			//!< Feature descriptor
				std::vector<float>			SpinImg;		//!< The 2D histogram as a single row
				uint16_t					SpinImg_range_rows;  //!< The number of rows (corresponding to range bins in the 2D histogram) of the original matrix from which SpinImg was extracted as a vector.
				mrpt::math::CMatrix			PolarImg;		//!< A polar image centered at the interest point
				mrpt::math::CMatrix			LogPolarImg;	//!< A log-polar image centered at the interest point
				bool						polarImgsNoRotation; //!< If set to true (manually, default=false) the call to "descriptorDistanceTo" will not consider all the rotations between polar image descriptors (PolarImg, LogPolarImg)

				bool hasDescriptorSIFT() const { return !SIFT.empty(); };          //!< Whether this feature has this kind of descriptor
				bool hasDescriptorSURF() const { return !SURF.empty(); }          //!< Whether this feature has this kind of descriptor
				bool hasDescriptorSpinImg() const { return !SpinImg.empty(); };       //!< Whether this feature has this kind of descriptor
				bool hasDescriptorPolarImg() const { return size(PolarImg,1)>0; } ;      //!< Whether this feature has this kind of descriptor
				bool hasDescriptorLogPolarImg() const { return size(LogPolarImg,1)>0; } ;      //!< Whether this feature has this kind of descriptor
			}
			descriptors;

			/** Return the first found descriptor, as a matrix.
			  * \return false on error, i.e. there is no valid descriptor.
			  */
			bool getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat &desc) const;

			/** Computes the normalized cross-correlation between the patches of this and another feature (normalized in the range [0,1], such as 0=best, 1=worst).
			  *  \note If this or the other features does not have patches or they are of different sizes, an exception will be raised.
			  * \sa descriptorDistanceTo
			  */
			float patchCorrelationTo( const CFeature &oFeature) const;

			/** Computes the Euclidean Distance between this feature's and other feature's descriptors, using the given descriptor or the first present one.
			  *  \note If descriptorToUse is not descAny and that descriptor is not present in one of the features, an exception will be raised.
			  * \sa patchCorrelationTo
			  */
			float descriptorDistanceTo( const CFeature &oFeature,  TDescriptorType descriptorToUse = descAny, bool normalize_distances = true ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSIFTDistanceTo( const CFeature &oFeature, bool normalize_distances = true ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSURFDistanceTo( const CFeature &oFeature, bool normalize_distances = true  ) const;

			/** Computes the Euclidean Distance between "this" and the "other" descriptors */
			float descriptorSpinImgDistanceTo( const CFeature &oFeature, bool normalize_distances = true ) const;

			/** Returns the minimum Euclidean Distance between "this" and the "other" polar image descriptor, for the best shift in orientation.
			  * \param oFeature The other feature to compare with.
			  * \param minDistAngle The placeholder for the angle at which the smallest distance is found.
			  * \return The distance for the best orientation (minimum distance).
			  */
			float descriptorPolarImgDistanceTo(
				const CFeature &oFeature,
				float &minDistAngle,
				bool normalize_distances = true ) const;

			/** Returns the minimum Euclidean Distance between "this" and the "other" log-polar image descriptor, for the best shift in orientation.
			  * \param oFeature The other feature to compare with.
			  * \param minDistAngle The placeholder for the angle at which the smallest distance is found.
			  * \return The distance for the best orientation (minimum distance).
			  */
			float descriptorLogPolarImgDistanceTo(
				const CFeature &oFeature,
				float &minDistAngle,
				bool normalize_distances = true ) const;

			/** Get the type of the feature
			*/
			TFeatureType get_type() const { return type; }

			/** Constructor
			*/
			CFeature();

			/** Virtual destructor */
			virtual ~CFeature() {}


		protected:
			//unsigned int numRefs;

			/** Internal function used by "descriptorLogPolarImgDistanceTo" and "descriptorPolarImgDistanceTo"
			  */
			static float internal_distanceBetweenPolarImages(
				const mrpt::math::CMatrix &desc1,
				const mrpt::math::CMatrix &desc2,
				float &minDistAngle,
				bool normalize_distances,
				bool dont_shift_angle );

		}; // end of class

		/****************************************************
						Class CFEATURELIST
		*****************************************************/
		/** A list of features
		*/
		class VISION_IMPEXP CFeatureList : public std::deque<CFeaturePtr>
		{
		public:

			/** The type of feature which contains
			*/
			TFeatureType get_type() const {
				if( size() > 0 )
					return (*begin())->get_type();
				else
					return featNotDefined;
			}

			/** Save feature list to a text file
			*/
			void saveToTextFile( const std::string &fileName, bool APPEND = false );

			/** Save feature list to a text file
			*/
			void loadFromTextFile( const std::string &fileName );

			/** Get the maximum ID into the list
			*/
			TFeatureID getMaxID() const;

			/** Get a reference to a Feature from its ID
			*/
			CFeaturePtr getByID( TFeatureID ID ) const;

			/** Get a reference of the nearest Feature to a given point
			* Use dist_prev to stablish the maximum distance of search, this parameter gives the distance of the nearest element
			*/
			CFeaturePtr nearest( const double &x, const double &y, double &dist_prev ) const;

			/** Constructor
			*/
			CFeatureList();

			/** Virtual destructor */
			virtual ~CFeatureList();

		}; // end of class

		/****************************************************
					Class CMATCHEDFEATURELIST
		*****************************************************/
		/** A list of features
		*/
		class VISION_IMPEXP CMatchedFeatureList : public std::deque< std::pair<CFeaturePtr,CFeaturePtr> >
		{
		public:
			/** The type of feature which contains
			*/
			TFeatureType get_type() const {
				if( size() > 0 )
					return (begin()->first)->get_type();
				else
					return featNotDefined;
			}

			/** Save list of matched features to a text file
			*/
			void saveToTextFile(const std::string &fileName);

			/** Constructor
			*/
			CMatchedFeatureList();

			/** Virtual destructor */
			virtual ~CMatchedFeatureList();
		}; // end of class

	} // end of namespace

	namespace utils
	{
		using namespace ::mrpt::vision;
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR(CFeature)
	}


} // end of namespace

#endif

