/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#include "CNode.h"

#include <stdexcept>

CNode::CNode(CNode* parent) : m_parent(parent) {}
const CNode* CNode::parentItem() const { return m_parent; }
void CNode::addNewChild()
{
	throw std::runtime_error("addNewChild not implemented!");
}

const CNode* CNode::child(int id) const
{
	return const_cast<CNode*>(this)->child(id);
}
