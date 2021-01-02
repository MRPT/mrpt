/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
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
