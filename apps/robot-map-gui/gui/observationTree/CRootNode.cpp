/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CRootNode.h"
#include "CPairNode.h"

using namespace mrpt;
using namespace mrpt::maps;

CRootNode::CRootNode(const CSimpleMap& simplemap) : CNode(nullptr)
{
	if (!simplemap.empty())
	{
		size_t idx = 0;
		for (auto& p : simplemap)
		{
			auto* node = new CPairNode(this, p, idx++);
			m_posesNode.push_back(node);
		}
	}
}

int CRootNode::childCount() const { return m_posesNode.size(); }
CNode* CRootNode::child(int id)
{
	int size = m_posesNode.size();
	ASSERT_(id <= size);
	return m_posesNode[id];
}

CNode::ObjectType CRootNode::type() const { return ObjectType::Root; }
std::string CRootNode::displayName() const { return "root"; }
