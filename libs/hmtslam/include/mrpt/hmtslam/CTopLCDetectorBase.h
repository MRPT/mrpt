/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/hmtslam/HMT_SLAM_common.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3DPDF.h>

namespace mrpt::hmtslam
{
/** The virtual base class for Topological Loop-closure Detectors; used in
 * HMT-SLAM
 *  \sa mrpt::slam::CHMTSLAM
 * \ingroup mrpt_hmtslam_grp
 */
class CTopLCDetectorBase
{
   protected:
	CHMTSLAM* m_hmtslam;

	/** Instances can be generated through a class factory only */
	CTopLCDetectorBase(CHMTSLAM* htmslam_obj) : m_hmtslam(htmslam_obj) {}

   public:
	/** A class factory, to be implemented in derived classes. */
	// static CTopLCDetectorBase* createNewInstance();

	/** Destructor */
	virtual ~CTopLCDetectorBase() = default;
	/** Reset the internal state of the TLCD, if any.
	 *  This is needed since the objects are created while loading HMT-SLAM
	 * options, but the algorithm may be re-started after that at any time.
	 */
	virtual void reset()
	{
		// By default, do nothing.
	}

	/** This method must compute the topological observation model.
	 * \param out_log_lik The output, a log-likelihood.
	 * \return nullptr (an empty smart pointer), or a PDF of the estimated
	 * translation between the two areas (can be a multi-modal PDF).
	 */
	virtual mrpt::poses::CPose3DPDF::Ptr computeTopologicalObservationModel(
		const THypothesisID& hypID, const CHMHMapNode::Ptr& currentArea,
		const CHMHMapNode::Ptr& refArea, double& out_log_lik) = 0;

	/** If implemented, this method provides the evaluation of an additional
	 * term to be added to the SSO between each pair of observations.
	 * \param out_SSO The output, in the range [0,1].
	 * \return true if computed SSO is meaningful. The default virtual method
	 * returns false.
	 */
	virtual bool computeSSOBetweenObservations(
		[[maybe_unused]] const THypothesisID& hypID,
		[[maybe_unused]] const TPoseID& poseID1,
		[[maybe_unused]] const TPoseID& poseID2,
		[[maybe_unused]] double& out_SSO)
	{
		return false;
	}

	/** Hook method for being warned about the insertion of a new poses into the
	 * maps.
	 *  This should be independent of hypothesis IDs.
	 */
	virtual void OnNewPose(
		[[maybe_unused]] const TPoseID& poseID,
		[[maybe_unused]] const mrpt::obs::CSensoryFrame* SF)
	{
	}
	using Ptr = std::shared_ptr<CTopLCDetectorBase>;

};	// end class

}  // namespace mrpt::hmtslam
