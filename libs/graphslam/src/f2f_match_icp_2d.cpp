/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphslam.h>
#include <mrpt/slam/CICP.h>

using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::f2f_match;

// Default ctor
GS_F2F_ICP_2D::GS_F2F_ICP_2D()
{
}

/** Keyframe-to-keyframe match method.
	* \return true if a valid registration was found.
	*/
bool GS_F2F_ICP_2D::matchTwoKeyframes(
	const TNodeID id_a, const TNodeID id_b, 
	const CSensoryFrame &obs_a, const CSensoryFrame &obs_b,
	const mrpt::poses::CPose2D &approx_pose_b_from_a,
	mrpt::poses::CPosePDFGaussianInf & out_pose_b_from_a)
{
	CICP	ICP;

	ICP.options = params.icp_params;

	const mrpt::slam::CPointsMap *map_a = obs_a.buildAuxPointsMap<mrpt::slam::CPointsMap>();
	const mrpt::slam::CPointsMap *map_b = obs_b.buildAuxPointsMap<mrpt::slam::CPointsMap>();

	// If none of the observations can be used to build a point cloud, we can't register these two keyframes!
	if (!map_a || !map_b || map_a->isEmpty() || map_b->isEmpty()) return false;

	CICP::TReturnInfo icpReturn;
	CPosePDFPtr pestPose= ICP.Align(
		map_a, // Map 1
		map_b, // Map 2
		approx_pose_b_from_a, // a first gross estimation of map 2 relative to map 1.
		NULL,      // Running time
		&icpReturn // Returned information
		);

	

	if (icpReturn.goodness> params.minICP_goodness_to_accept)
	{
		// save estimation:
		out_pose_b_from_a.copyFrom( *pestPose );
#ifdef _DEBUG
		std::cout << "[GS_F2F_ICP_2D] Match FOUND " << id_a << "-" << id_b << ": goodness=" << icpReturn.goodness << " rel.pose=" << out_pose_b_from_a << std::endl;
#endif
		return true;
	}
	else
	{
#ifdef _DEBUG
		std::cout << "[GS_F2F_ICP_2D] NO Match " << id_a << "-" << id_b << ": goodness=" << icpReturn.goodness << std::endl;
#endif
		return false;
	}
}

bool GS_F2F_ICP_2D::matchTwoKeyframes(
	const TNodeID id_a, const TNodeID id_b, const CSensoryFrame &obs_a, const CSensoryFrame &obs_b,
	const mrpt::poses::CPose2D &approx_pose_b_from_a, mrpt::poses::CPose2D &out_pose_b_from_a )
{
	mrpt::poses::CPosePDFGaussianInf out_pose_b_from_a_;
	bool valid = this->matchTwoKeyframes(id_a,id_b,obs_a,obs_b,approx_pose_b_from_a,out_pose_b_from_a_);
	out_pose_b_from_a = out_pose_b_from_a_.mean;
	return valid;
}


// TParams:
GS_F2F_ICP_2D::TParams::TParams() :
	kf2kf_max_search_radius (6.0),
	minICP_goodness_to_accept (0.50)
{
}

void GS_F2F_ICP_2D::TParams::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR(kf2kf_max_search_radius, double, source,section)
	MRPT_LOAD_CONFIG_VAR(minICP_goodness_to_accept, double, source,section)

	icp_params.loadFromConfigFile(source,section);
}

void GS_F2F_ICP_2D::TParams::dumpToTextStream(mrpt::utils::CStream &out) const
{
	out.printf("\n----------- [GS_F2F_ICP_2D::TParams] ------------ \n\n");
	LOADABLEOPTS_DUMP_VAR(kf2kf_max_search_radius, double)
	LOADABLEOPTS_DUMP_VAR(minICP_goodness_to_accept, double)

	icp_params.dumpToTextStream(out);
}

		