/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CPosesNode.h"

#include "mrpt/poses/CPose3DPDF.h"


using namespace mrpt;
using namespace mrpt::poses;

CPoseNode::CPoseNode(CNode *parent, const CPose3D& pose)
	: CNode(parent)
	, m_pose(pose)
{

}

int CPoseNode::childCount() const
{
	return 0;
}


CNode *CPoseNode::child(int id)
{
	return nullptr;
}


CPose3D CPoseNode::getPose() const
{
	return m_pose;
}

std::string CPoseNode::displayName() const
{
	std::string str;
	m_pose.asString(str);
	return "Pose: " + str;
}


CNode::ObjectType CPoseNode::type() const
{
	return ObjectType::Pos;
}
