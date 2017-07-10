/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CObservationsNode.h"

#include "CRangeScanNode.h"
#include "CBaseObservationNode.h"

#include "mrpt/obs/CObservation2DRangeScan.h"


using namespace mrpt;
using namespace mrpt::obs;

CObservationsNode::CObservationsNode(CNode *parent, const CSensoryFrame::Ptr &sensoryFrame, const poses::CPose3D &pose)
	: CNode(parent)
{
	auto scanPtr = sensoryFrame->getObservationByClass<CObservation2DRangeScan>();
	if (scanPtr)
	{
		m_observations.emplace_back(std::make_unique<CRangeScanNode>(this, scanPtr, pose));
	}
}

CObservationsNode::~CObservationsNode()
{
}

int CObservationsNode::childCount() const
{
	return m_observations.size();
}

CNode *CObservationsNode::child(int id)
{
	return static_cast<CNode *>(m_observations[id].get());
}

CNode::ObjectType CObservationsNode::type() const
{
	return ObjectType::Observations;
}

std::string CObservationsNode::displayName() const
{
	return "Observations";
}
