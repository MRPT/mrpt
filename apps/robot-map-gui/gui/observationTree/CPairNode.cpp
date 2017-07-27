/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CPairNode.h"

#include "CPosesNode.h"
#include "CObservationsNode.h"

using namespace mrpt;
using namespace mrpt::maps;

CPairNode::CPairNode(
	CNode* parent, const CSimpleMap::TPosePDFSensFramePair& poseSensFramePair)
	: CNode(parent)
{
	mrpt::poses::CPose3D pos = poseSensFramePair.first->getMeanVal();
	m_pose = std::make_unique<CPosesNode>(this, pos);
	m_observations = std::make_unique<CObservationsNode>(
		this, poseSensFramePair.second, pos);
}

CPairNode::~CPairNode() {}
int CPairNode::childCount() const { return 2; }
CNode* CPairNode::child(int id) { return getChild(id); }
CNode* CPairNode::child(int id) const { return getChild(id); }
CNode::ObjectType CPairNode::type() const
{
	return ObjectType::PosWithObservationPair;
}

std::string CPairNode::displayName() const { return "Position"; }
CNode* CPairNode::getChild(int id) const
{
	switch (id)
	{
		case 0:
			return m_pose.get();
		case 1:
			return m_observations.get();
		default:
			return nullptr;
	};
}
