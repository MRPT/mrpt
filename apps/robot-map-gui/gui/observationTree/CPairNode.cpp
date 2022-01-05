/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CPairNode.h"

#include "CObservationsNode.h"
#include "CPosesNode.h"

using namespace mrpt;
using namespace mrpt::maps;

CPairNode::CPairNode(
	CNode* parent, const CSimpleMap::Pair& poseSensFramePair,
	size_t indexInSimpleMap)
	: CNode(parent), m_indexInSimpleMap(indexInSimpleMap)
{
	mrpt::poses::CPose3D pos = poseSensFramePair.pose->getMeanVal();
	m_pose = std::make_unique<CPosesNode>(this, pos);
	m_observations =
		std::make_unique<CObservationsNode>(this, poseSensFramePair.sf, pos);
}

CPairNode::~CPairNode() = default;
int CPairNode::childCount() const { return 2; }
CNode* CPairNode::child(int id) { return getChild(id); }
CNode* CPairNode::child(int id) const { return getChild(id); }
CNode::ObjectType CPairNode::type() const
{
	return ObjectType::PosWithObservationPair;
}

std::string CPairNode::displayName() const
{
	return std::string("[") + std::to_string(m_indexInSimpleMap) +
		std::string("] Pose-SF pair");
}
CNode* CPairNode::getChild(int id) const
{
	switch (id)
	{
		case 0: return m_pose.get();
		case 1: return m_observations.get();
		default: return nullptr;
	};
}
