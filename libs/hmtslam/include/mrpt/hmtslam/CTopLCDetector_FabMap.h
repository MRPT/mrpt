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

namespace mrpt::hmtslam
{
/** \ingroup mrpt_hmtslam_grp */
class CTopLCDetector_FabMap : public CTopLCDetectorBase
{
   protected:
	CTopLCDetector_FabMap(CHMTSLAM* hmtslam);

	void* m_fabmap;  // FabMapInstance*

   public:
	/** A class factory, to be implemented in derived classes.
	 */
	static CTopLCDetectorBase* createNewInstance(CHMTSLAM* hmtslam)
	{
		return static_cast<CTopLCDetectorBase*>(
			new CTopLCDetector_FabMap(hmtslam));
	}

	/** Destructor */
	~CTopLCDetector_FabMap() override;

	/** This method must compute the topological observation model.
	 * \param out_log_lik The output, a log-likelihood.
	 * \return nullptr (empty smart pointer), or a PDF of the estimated
	 * translation between the two areas (can be a multi-modal PDF).
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

	/** Options for a TLC-detector of type FabMap, used from CHMTSLAM
	 */
	struct TOptions : public mrpt::config::CLoadableOptions
	{
		/** Initialization of default params
		 */
		TOptions();

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;  // See base docs
		void dumpToTextStream(
			std::ostream& out) const override;  // See base docs

		std::string vocab_path, vocabName;
		double p_obs_given_exists{0.39}, p_at_new_place{0.99},
			df_lik_smooth{0.99};
	};

};  // end class
}  // namespace mrpt::hmtslam
