/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CObservationsNode.h"

#include "CRangeScanNode.h"
#include "CObservationImageNode.h"
#include "CObservationStereoImageNode.h"
#include "CBaseObservationNode.h"

#include "mrpt/obs/CObservation2DRangeScan.h"
#include "mrpt/obs/CObservationImage.h"
#include "mrpt/obs/CObservationStereoImages.h"

using namespace mrpt;
using namespace mrpt::obs;

CObservationsNode::CObservationsNode(
	CNode* parent, const CSensoryFrame::Ptr& sensoryFrame,
	const poses::CPose3D& pose)
	: CNode(parent)
{
	auto scanRangeScan =
		sensoryFrame->getObservationByClass<CObservation2DRangeScan>();
	if (scanRangeScan)
		m_observations.emplace_back(
			std::make_unique<CRangeScanNode>(this, scanRangeScan, pose));

	auto scanImg = sensoryFrame->getObservationByClass<CObservationImage>();
	if (scanImg)
		m_observations.emplace_back(
			std::make_unique<CObservationImageNode>(this, scanImg, pose));

	auto scanStereo =
		sensoryFrame->getObservationByClass<CObservationStereoImages>();
	if (scanStereo)
		m_observations.emplace_back(
			std::make_unique<CObservationStereoImagesNode>(
				this, scanStereo, pose));
}

CObservationsNode::~CObservationsNode() = default;
int CObservationsNode::childCount() const { return m_observations.size(); }
CNode* CObservationsNode::child(int id)
{
	return static_cast<CNode*>(m_observations[id].get());
}

CNode::ObjectType CObservationsNode::type() const
{
	return ObjectType::Observations;
}

std::string CObservationsNode::displayName() const { return "Observations"; }
