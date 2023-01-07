/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"

using namespace mrpt;
using namespace mrpt::poses;

CPosesNode::CPosesNode(CNode* parent, const CPose3D& pose)
	: CNode(parent), m_pose(pose)
{
}

int CPosesNode::childCount() const { return 0; }
CNode* CPosesNode::child(int id) { return nullptr; }
CPose3D CPosesNode::getPose() const { return m_pose; }
std::string CPosesNode::displayName() const
{
	using namespace std::string_literals;
	return "Pose: "s + m_pose.asString();
}

CNode::ObjectType CPosesNode::type() const { return ObjectType::Pos; }
