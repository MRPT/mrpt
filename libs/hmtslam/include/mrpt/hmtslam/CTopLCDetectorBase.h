/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  _CTopLCDetectorBase_H
#define  _CTopLCDetectorBase_H

#include <mrpt/hmtslam/HMT_SLAM_common.h>

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/obs/CSensoryFrame.h>


namespace mrpt
{
	namespace hmtslam
	{
		/** The virtual base class for Topological Loop-closure Detectors; used in HMT-SLAM
		  *  \sa mrpt::slam::CHMTSLAM
		  * \ingroup mrpt_hmtslam_grp
		  */
		class HMTSLAM_IMPEXP CTopLCDetectorBase
		{
		protected:
			CHMTSLAM	*m_hmtslam;

			/** Instances can be generated through a class factory only */
			CTopLCDetectorBase( CHMTSLAM *htmslam_obj ) : m_hmtslam(htmslam_obj) {  }

		public:
			/** A class factory, to be implemented in derived classes. */
			//static CTopLCDetectorBase* createNewInstance();

			/** Destructor */
			virtual ~CTopLCDetectorBase() { }

			/** Reset the internal state of the TLCD, if any.
			  *  This is needed since the objects are created while loading HMT-SLAM options, but the algorithm may be re-started after that at any time.
			  */
			virtual void reset() {
				// By default, do nothing.
			}

			/** This method must compute the topological observation model.
			  * \param out_log_lik The output, a log-likelihood.
			  * \return NULL (an empty smart pointer), or a PDF of the estimated translation between the two areas (can be a multi-modal PDF).
			  */
			virtual mrpt::poses::CPose3DPDFPtr computeTopologicalObservationModel(
				const THypothesisID		&hypID,
				const CHMHMapNodePtr	&currentArea,
				const CHMHMapNodePtr	&refArea,
				double					&out_log_lik
				 ) = 0;

			/** If implemented, this method provides the evaluation of an additional term to be added to the SSO between each pair of observations.
			  * \param out_SSO The output, in the range [0,1].
			  * \return true if computed SSO is meaningful. The default virtual method returns false.
			  */
			virtual bool computeSSOBetweenObservations(
				const THypothesisID		&hypID,
				const TPoseID 			&poseID1,
				const TPoseID 			&poseID2,
				double					&out_SSO
				)
			{
				MRPT_UNUSED_PARAM(hypID); MRPT_UNUSED_PARAM(poseID1);
				MRPT_UNUSED_PARAM(poseID2); MRPT_UNUSED_PARAM(out_SSO);
				return false;
			}

			/** Hook method for being warned about the insertion of a new poses into the maps.
			  *  This should be independent of hypothesis IDs.
			  */
			virtual void OnNewPose(
				const TPoseID 			&poseID,
				const mrpt::obs::CSensoryFrame		*SF )
			{
				MRPT_UNUSED_PARAM(poseID); MRPT_UNUSED_PARAM(SF);
			}

		}; // end class

		typedef stlplus::smart_ptr<CTopLCDetectorBase> CTopLCDetectorBasePtr;

	} // end namespace
} // end namespace
#endif
