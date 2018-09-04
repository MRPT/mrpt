/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <string>
#include <vector>

#include "CNode.h"

#include "mrpt/maps/CSimpleMap.h"

class CRootNode : public CNode
{
   public:
	CRootNode(const mrpt::maps::CSimpleMap& simplemap);
	~CRootNode() override = default;

	// INode interface
	int childCount() const override;
	CNode* child(int id) override;
	ObjectType type() const override;
	std::string displayName() const override;

   private:
	std::vector<CNode*> m_posesNode;
};
