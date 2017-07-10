/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CRangeScanNode.h"


using namespace mrpt;
using namespace mrpt::obs;

CRangeScanNode::CRangeScanNode(CNode *parent, CObservation2DRangeScan::Ptr observation, const poses::CPose3D &pose)
	: CBaseObservationNode(parent)
	, m_observation(observation)
	, m_pose(pose)
{

}

CNode::ObjectType CRangeScanNode::type() const
{
	return ObjectType::RangeScan;
}

std::string CRangeScanNode::displayName() const
{
	return "Range scan";
}

CObservation2DRangeScan::Ptr CRangeScanNode::observation() const
{
	return m_observation;
}

poses::CPose3D CRangeScanNode::getPose() const
{
	return m_pose;
}
