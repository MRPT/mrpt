/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hmtslam/CTopLCDetectorBase.h>
#include <mrpt/slam/CGridMapAligner.h>

namespace mrpt::hmtslam
{
/** \ingroup mrpt_hmtslam_grp */
class CTopLCDetector_GridMatching : public CTopLCDetectorBase
{
   protected:
	CTopLCDetector_GridMatching(CHMTSLAM* hmtslam);

   public:
	/** A class factory, to be implemented in derived classes.
	 */
	static CTopLCDetectorBase* createNewInstance(CHMTSLAM* hmtslam)
	{
		return static_cast<CTopLCDetectorBase*>(
			new CTopLCDetector_GridMatching(hmtslam));
	}

	/** Destructor */
	~CTopLCDetector_GridMatching() override;

	/** This method must compute the topological observation model.
	 * \param out_log_lik The output, a log-likelihood.
	 * \return nullptr, or a PDF of the estimated translation between the two
	 * areas (should be a SOG PDF): it's the pose of "refArea", relative to
	 * "currentArea".
	 */
	mrpt::poses::CPose3DPDF::Ptr computeTopologicalObservationModel(
		const THypothesisID& hypID, const CHMHMapNode::Ptr& currentArea,
		const CHMHMapNode::Ptr& refArea, double& out_log_lik) override;

	/** Hook method for being warned about the insertion of a new poses into the
	 * maps.
	 *  This should be independent of hypothesis IDs.
	 */
	void OnNewPose(
		const TPoseID& poseID, const mrpt::obs::CSensoryFrame* SF) override;

	/** Options for a TLC-detector of type gridmap-matching, used from CHMTSLAM
	 */
	struct TOptions : public mrpt::config::CLoadableOptions
	{
		/** Initialization of default params
		 */
		TOptions();

		/** Options for the grid-to-grid matching algorithm */
		mrpt::slam::CGridMapAligner::TConfigParams matchingOptions;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs
	};
};  // end class
}  // namespace mrpt::hmtslam
