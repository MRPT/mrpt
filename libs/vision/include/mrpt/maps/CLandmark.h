/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CLandmark_H
#define CLandmark_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/system/datetime.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace maps
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLandmark, mrpt::utils::CSerializable, VISION_IMPEXP )

		/** The class for storing "landmarks" (visual or laser-scan-extracted features,...)
		  *
		  *  The descriptors for each kind of descriptor are stored in the vector "features", which
		  *   will typically consists of only 1 element, or 2 elements for landmarks obtained from stereo images.
		  *
		  * \sa CLandmarksMap
	 	  * \ingroup mrpt_vision_grp
		  */
		class VISION_IMPEXP CLandmark : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CLandmark )

		public:
			typedef	int64_t TLandmarkID;					//!< The type for the IDs of landmarks.

			std::vector<mrpt::vision::CFeaturePtr> features;		//!< The set of features from which the landmark comes.

			mrpt::math::TPoint3D pose_mean;					//!< The mean of the landmark 3D position.
			mrpt::math::TPoint3D normal;					//!< The "normal" to the landmark, i.e. a unitary 3D vector towards the viewing direction, or a null vector if not applicable
			float	pose_cov_11,pose_cov_22,pose_cov_33,pose_cov_12,pose_cov_13,pose_cov_23;

			/** An ID for the landmark (see details next...)
			  *  This ID was introduced in the version 3 of this class (21/NOV/2006), and its aim is
			  *  to provide a way for easily establishing correspondences between landmarks detected
			  *  in sequential image frames. Thus, the management of this field should be:
			  *		- In 'servers' (classes/modules/... that detect landmarks from images): A different ID must be assigned to every landmark (e.g. a sequential counter), BUT only in the case of being sure of the correspondence of one landmark with another one in the past (e.g. tracking).
			  *		- In 'clients': This field can be ignored, but if it is used, the advantage is solving the correspondence between landmarks detected in consequentive instants of time: Two landmarks with the same ID <b>correspond</b> to the same physical feature, BUT it should not be expected the inverse to be always true.
			  *
			  * Note that this field is never fill out automatically, it must be set by the programmer if used.
			  */
			TLandmarkID					ID;
			mrpt::system::TTimeStamp	timestampLastSeen;	//!< The last time that this landmark was observed.
			uint32_t					seenTimesCount;		//!< The number of times that this landmark has been seen.

			/** Returns the pose as an object:
			  */
			void 	getPose( mrpt::poses::CPointPDFGaussian &p ) const;

			void 	getPose( mrpt::poses::CPoint3D &p, mrpt::math::CMatrixDouble &COV ) const {
				mrpt::poses::CPointPDFGaussian pdf;
				getPose(pdf);
				p = pdf.mean;
				COV = mrpt::math::CMatrixDouble(pdf.cov);
			}

			/** Sets the pose from an object:
			  */
			void 	setPose( const mrpt::poses::CPointPDFGaussian &p );

			/** Gets the type of the first feature in its feature vector. The vector must not be empty.
			  */
			mrpt::vision::TFeatureType getType() const
			{ ASSERT_( !features.empty() ); ASSERT_(features[0].present()) return features[0]->type; }

			/** Creates one feature in the vector "features", calling the appropriate constructor of the smart pointer, so after calling this method "features[0]" is a valid pointer to a CFeature object.
			  */
			void createOneFeature()
			{ features.assign(1, mrpt::vision::CFeaturePtr( new mrpt::vision::CFeature() ) ); }

			/** Default constructor
			  */
			CLandmark();

			/** Virtual destructor
			  */
			virtual ~CLandmark();

		protected:
			/** Auxiliary variable
			  */
			static TLandmarkID		m_counterIDs;

		}; // End of class definition
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLandmark, mrpt::utils::CSerializable, VISION_IMPEXP )

	} // End of namespace
} // End of namespace

#endif
