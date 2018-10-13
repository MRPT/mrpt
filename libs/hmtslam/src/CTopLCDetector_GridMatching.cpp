/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h"  // Precomp header

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::maps;

CTopLCDetector_GridMatching::CTopLCDetector_GridMatching(CHMTSLAM* hmtslam)
	: CTopLCDetectorBase(hmtslam)
{
}

CTopLCDetector_GridMatching::~CTopLCDetector_GridMatching() = default;
/** This method must compute the topological observation model.
 * \param out_log_lik The output, a log-likelihood.
 * \return nullptr, or a PDF of the estimated translation between the two areas
 * (can be a multi-modal PDF).
 */
CPose3DPDF::Ptr CTopLCDetector_GridMatching::computeTopologicalObservationModel(
	const THypothesisID& hypID, const CHMHMapNode::Ptr& currentArea,
	const CHMHMapNode::Ptr& refArea, double& out_log_lik)
{
	// ------------------------------------------------------------------------------
	// Compute potential map transformations between the areas "areaID" and
	// "a->first"
	//  For this, we use the grid-to-grid matching method described in the TRO
	//  paper...
	// ------------------------------------------------------------------------------
	out_log_lik = 0;  // Nothing to modify, for now.

	CGridMapAligner gridAligner;
	CGridMapAligner::TReturnInfo info;
	const CPose2D initEstimate(
		0, 0, 0);  // It's actually ignored by the grid-matching

	// Use already loaded options:
	const CTopLCDetector_GridMatching::TOptions& o =
		m_hmtslam->m_options.TLC_grid_options;
	gridAligner.options = o.matchingOptions;

	CMultiMetricMap::Ptr hMapCur =
		currentArea->m_annotations.getAs<CMultiMetricMap>(
			NODE_ANNOTATION_METRIC_MAPS, hypID, false);
	CMultiMetricMap::Ptr hMapRef =
		refArea->m_annotations.getAs<CMultiMetricMap>(
			NODE_ANNOTATION_METRIC_MAPS, hypID, false);

	ASSERT_(hMapRef->m_gridMaps.size() >= 1);
	ASSERT_(hMapCur->m_gridMaps.size() >= 1);

#if 0
	{
		static int i = 0;
		CFileOutputStream f(format("debug_%05i.hmtslam",++i));
		f << *m_hmtslam;
	}
#endif

	gridAligner.options.dumpToConsole();

	// Do the map align:
	CPosePDF::Ptr alignRes = gridAligner.Align(
		hMapCur.get(),  // "ref" as seen from "cur"...The order is critical!!!
		hMapRef.get(), initEstimate, nullptr, &info);

#if 0
	{
		hMapCur->m_gridMaps[0]->saveAsBitmapFileWithLandmarks("map1.png", info.landmarks_map1.get());
		hMapRef->m_gridMaps[0]->saveAsBitmapFileWithLandmarks("map2.png", info.landmarks_map2.get());
	}
#endif

	// Transform the 2D SOG into a 3D SOG:
	CPose3DPDF::Ptr res = CPose3DPDF::Ptr(CPose3DPDF::createFrom2D(*alignRes));

// --------------------
// Debug output:
// --------------------
#if 1
	const std::string dbg_dir =
		m_hmtslam->m_options.LOG_OUTPUT_DIR + "/TBI_TESTS";
	if (!m_hmtslam->m_options.LOG_OUTPUT_DIR.empty())
	{
		mrpt::system::createDirectory(dbg_dir);
		static int cnt = 0;
		++cnt;
		const std::string filStat =
			dbg_dir + format(
						  "/state_%05i_test_%i_%i.hmtslam", cnt,
						  (int)currentArea->getID(), (int)refArea->getID());
		const std::string filRes =
			dbg_dir + format(
						  "/state_%05i_test_%i_%i_result.txt", cnt,
						  (int)currentArea->getID(), (int)refArea->getID());

		m_hmtslam->logFmt(
			mrpt::system::LVL_DEBUG, "[TLCD_gridmatch] DEBUG: Saving %s\n",
			filStat.c_str());
		CFileGZOutputStream f(filStat);
		auto arch = mrpt::serialization::archiveFrom(f);
		this->m_hmtslam->saveState(arch);

		m_hmtslam->logFmt(
			mrpt::system::LVL_DEBUG, "[TLCD_gridmatch] DEBUG: Saving %s\n",
			filRes.c_str());
		CFileOutputStream f_res(filRes);
		f_res.printf(
			"# SOG modes: %i\n",
			(int)std::dynamic_pointer_cast<CPosePDFSOG>(alignRes)->size());
		f_res.printf("ICP goodness: ");
		f_res.printf_vector("%f ", info.icp_goodness_all_sog_modes);
		f_res.printf("\n");
	}
#endif

	return res;
}

/** Hook method for being warned about the insertion of a new poses into the
 * maps.
 *  This should be independent of hypothesis IDs.
 */
void CTopLCDetector_GridMatching::OnNewPose(
	const TPoseID& poseID, const CSensoryFrame* SF)
{
	MRPT_UNUSED_PARAM(poseID);
	MRPT_UNUSED_PARAM(SF);
}

// Initialization
CTopLCDetector_GridMatching::TOptions::TOptions() : matchingOptions() {}
//  Load parameters from configuration source
void CTopLCDetector_GridMatching::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	matchingOptions.loadFromConfigFile(source, section);
}

//  This method must display clearly all the contents of the structure in
//  textual form, sending it to a CStream.
void CTopLCDetector_GridMatching::TOptions::dumpToTextStream(
	std::ostream& out) const
{
	out << mrpt::format(
		"\n----------- [CTopLCDetector_GridMatching::TOptions] ------------ "
		"\n\n");
	matchingOptions.dumpToTextStream(out);
}
