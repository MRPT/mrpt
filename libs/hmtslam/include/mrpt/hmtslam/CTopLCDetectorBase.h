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
