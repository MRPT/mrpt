/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CObservationImageNode.h"

CObservationImageNode::CObservationImageNode(
	CNode* parent, mrpt::obs::CObservationImage::Ptr observation,
	const mrpt::poses::CPose3D& pose)
	: CBaseObservationNode(parent, pose), m_observation(observation)
{
}

CNode::ObjectType CObservationImageNode::type() const
{
	return ObjectType::Image;
}

std::string CObservationImageNode::displayName() const { return "Image"; }
mrpt::obs::CObservationImage::Ptr CObservationImageNode::observation() const
{
	return m_observation;
}
