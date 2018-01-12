/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CObservationStereoImageNode.h"

CObservationStereoImagesNode::CObservationStereoImagesNode(
	CNode* parent, mrpt::obs::CObservationStereoImages::Ptr observation,
	const mrpt::poses::CPose3D& pose)
	: CBaseObservationNode(parent, pose), m_observation(observation)
{
}

CNode::ObjectType CObservationStereoImagesNode::type() const
{
	return ObjectType::StereoImage;
}

std::string CObservationStereoImagesNode::displayName() const
{
	return "Stereo Image";
}

mrpt::obs::CObservationStereoImages::Ptr
	CObservationStereoImagesNode::observation() const
{
	return m_observation;
}
