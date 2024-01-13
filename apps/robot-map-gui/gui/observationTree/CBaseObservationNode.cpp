/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CBaseObservationNode.h"

CBaseObservationNode::CBaseObservationNode(
	CNode* parent, const mrpt::poses::CPose3D& pose)
	: CNode(parent), m_pose(pose)
{
}

int CBaseObservationNode::childCount() const { return 0; }
CNode* CBaseObservationNode::child(int id) { return nullptr; }
mrpt::poses::CPose3D CBaseObservationNode::getPose() const { return m_pose; }
