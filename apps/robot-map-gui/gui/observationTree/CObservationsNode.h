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

#include "mrpt/obs/CSensoryFrame.h"

#include <vector>

class CBaseObservationNode;

class CObservationsNode : public CNode
{
   public:
	CObservationsNode(
		CNode* parent, const mrpt::obs::CSensoryFrame::Ptr& sensoryFrame,
		const mrpt::poses::CPose3D& pose);
	~CObservationsNode() override;
	// CNode interface
	int childCount() const override;
	CNode* child(int id) override;
	ObjectType type() const override;
	std::string displayName() const override;

   private:
	std::vector<std::unique_ptr<CBaseObservationNode>> m_observations;
};
