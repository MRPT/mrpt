/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef  _CTopLCDetectorBase_H
#define  _CTopLCDetectorBase_H

#include <mrpt/hmtslam/HMT_SLAM_common.h>

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/slam/CSensoryFrame.h>


namespace mrpt
{
	namespace hmtslam
	{
		using namespace mrpt::slam;

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
			virtual CPose3DPDFPtr computeTopologicalObservationModel(
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
				return false;
			}

			/** Hook method for being warned about the insertion of a new poses into the maps.
			  *  This should be independent of hypothesis IDs.
			  */
			virtual void OnNewPose(
				const TPoseID 			&poseID,
				const CSensoryFrame		*SF )
			{ }

		}; // end class

		typedef stlplus::smart_ptr<CTopLCDetectorBase> CTopLCDetectorBasePtr;

	} // end namespace
} // end namespace
#endif
