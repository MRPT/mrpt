/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CLandmark_H
#define CLandmark_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/math/lightweight_geom_data.h>


namespace mrpt
{
	/** \ingroup mrpt_vision_grp */
	namespace slam
	{
		using namespace mrpt::poses;
		using namespace mrpt::vision;
		using namespace mrpt::math;

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

			std::vector<CFeaturePtr> features;		//!< The set of features from which the landmark comes.

			TPoint3D pose_mean;					//!< The mean of the landmark 3D position.
			TPoint3D normal;					//!< The "normal" to the landmark, i.e. a unitary 3D vector towards the viewing direction, or a null vector if not applicable
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
			void 	getPose( CPointPDFGaussian &p ) const;

			void 	getPose( CPoint3D &p, CMatrixDouble &COV ) const {
				CPointPDFGaussian pdf;
				getPose(pdf);
				p = pdf.mean;
				COV = CMatrixDouble(pdf.cov);
			}

			/** Sets the pose from an object:
			  */
			void 	setPose( const CPointPDFGaussian &p );

			/** Gets the type of the first feature in its feature vector. The vector must not be empty.
			  */
			TFeatureType getType() const
			{ ASSERT_( !features.empty() ); ASSERT_(features[0].present()) return features[0]->type; }

			/** Creates one feature in the vector "features", calling the appropriate constructor of the smart pointer, so after calling this method "features[0]" is a valid pointer to a CFeature object.
			  */
			void createOneFeature()
			{ features.assign(1, CFeaturePtr( new CFeature() ) ); }

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

	} // End of namespace
} // End of namespace

#endif
