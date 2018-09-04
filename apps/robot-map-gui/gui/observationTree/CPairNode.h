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
#include "CNode.h"

#include <string>

#include "mrpt/maps/CSimpleMap.h"

class CObservationsNode;
class CPosesNode;

class CPairNode : public CNode
{
   public:
	CPairNode(
		CNode* parent,
		const mrpt::maps::CSimpleMap::TPosePDFSensFramePair& poseSensFramePair,
		size_t indexInSimpleMap);
	~CPairNode() override;

	int childCount() const override;

	CNode* child(int id) override;
	CNode* child(int id) const;
	ObjectType type() const override;
	std::string displayName() const override;

   private:
	CNode* getChild(int id) const;

	std::unique_ptr<CPosesNode> m_pose;
	std::unique_ptr<CObservationsNode> m_observations;
	size_t m_indexInSimpleMap;
};
